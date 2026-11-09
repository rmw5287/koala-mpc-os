// Step 1: framebuffer and evdev hardware diagnostic for the MPC Sample.
// Build: arm-linux-gnueabihf-g++ -O3 -std=c++17 main.cpp -lpthread -o firmware.bin
// Run from the device's Linux shell (typically as root for /dev/fb0 and /dev/input access).

#include <algorithm>
#include <array>
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <dirent.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <memory>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <linux/input.h>
#include <unistd.h>
#include <vector>

namespace {
volatile std::sig_atomic_t g_running = 1;
void stop(int) { g_running = 0; }

// A compact 5x7 uppercase bitmap font, embedded so the diagnostic has no
// dependency on font files, SDL, X11, or other graphics libraries.
std::array<uint8_t, 7> glyph(char c) {
    switch (c) {
        case 'A': return {14,17,17,31,17,17,17}; case 'B': return {30,17,17,30,17,17,30};
        case 'C': return {14,17,16,16,16,17,14}; case 'D': return {30,17,17,17,17,17,30};
        case 'E': return {31,16,16,30,16,16,31}; case 'F': return {31,16,16,30,16,16,16};
        case 'G': return {14,17,16,23,17,17,15}; case 'H': return {17,17,17,31,17,17,17};
        case 'I': return {14,4,4,4,4,4,14}; case 'J': return {7,2,2,2,18,18,12};
        case 'K': return {17,18,20,24,20,18,17}; case 'L': return {16,16,16,16,16,16,31};
        case 'M': return {17,27,21,21,17,17,17}; case 'N': return {17,25,21,19,17,17,17};
        case 'O': return {14,17,17,17,17,17,14}; case 'P': return {30,17,17,30,16,16,16};
        case 'Q': return {14,17,17,17,21,18,13}; case 'R': return {30,17,17,30,20,18,17};
        case 'S': return {15,16,16,14,1,1,30}; case 'T': return {31,4,4,4,4,4,4};
        case 'U': return {17,17,17,17,17,17,14}; case 'V': return {17,17,17,17,17,10,4};
        case 'W': return {17,17,17,21,21,21,10}; case 'X': return {17,17,10,4,10,17,17};
        case 'Y': return {17,17,10,4,4,4,4}; case 'Z': return {31,1,2,4,8,16,31};
        case '0': return {14,17,19,21,25,17,14}; case '1': return {4,12,4,4,4,4,14};
        case '2': return {14,17,1,2,4,8,31}; case '3': return {30,1,1,14,1,1,30};
        case '4': return {2,6,10,18,31,2,2}; case '5': return {31,16,16,30,1,1,30};
        case '6': return {14,16,16,30,17,17,14}; case '7': return {31,1,2,4,8,8,8};
        case '8': return {14,17,17,14,17,17,14}; case '9': return {14,17,17,15,1,1,14};
        case ':': return {0,4,4,0,4,4,0}; case '.': return {0,0,0,0,0,12,12};
        case ',': return {0,0,0,0,4,4,8}; case '-': return {0,0,0,31,0,0,0};
        case '_': return {0,0,0,0,0,0,31}; case '/': return {1,2,2,4,8,8,16};
        case '+': return {0,4,4,31,4,4,0}; case '=': return {0,31,0,31,0,0,0};
        case '[': return {14,8,8,8,8,8,14}; case ']': return {14,2,2,2,2,2,14};
        case '(': return {2,4,8,8,8,4,2}; case ')': return {8,4,2,2,2,4,8};
        case '#': return {10,31,10,10,31,10,0}; case '*': return {0,21,14,31,14,21,0};
        case ' ': return {0,0,0,0,0,0,0};
        default: return {31,17,1,2,4,0,4}; // visible placeholder for unsupported characters
    }
}

uint32_t channelBits(uint8_t value, const fb_bitfield& field) {
    if (field.length == 0) return 0;
    const uint64_t maxValue = (field.length >= 32) ? 0xffffffffULL : ((1ULL << field.length) - 1ULL);
    uint64_t scaled = (static_cast<uint64_t>(value) * maxValue + 127) / 255;
    if (field.msb_right) {
        uint64_t reversed = 0;
        for (uint32_t i = 0; i < field.length; ++i) reversed = (reversed << 1) | ((scaled >> i) & 1ULL);
        scaled = reversed;
    }
    return static_cast<uint32_t>(scaled << field.offset);
}

class Framebuffer {
public:
    bool openDevice(const char* path) {
        fd_ = ::open(path, O_RDWR | O_CLOEXEC);
        if (fd_ < 0) { error_ = std::string("OPEN ") + path + ": " + std::strerror(errno); return false; }
        if (ioctl(fd_, FBIOGET_FSCREENINFO, &fix_) < 0 || ioctl(fd_, FBIOGET_VSCREENINFO, &var_) < 0) {
            error_ = std::string("FBIOGET INFO: ") + std::strerror(errno); return false;
        }
        if ((var_.bits_per_pixel != 16 && var_.bits_per_pixel != 24 && var_.bits_per_pixel != 32) ||
            fix_.line_length == 0 || fix_.smem_len == 0) {
            error_ = "Unsupported framebuffer format (need 16, 24, or 32 bpp)"; return false;
        }
        width_ = var_.xres; height_ = var_.yres;
        mappingSize_ = fix_.smem_len;
        pixels_ = static_cast<uint8_t*>(mmap(nullptr, mappingSize_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
        if (pixels_ == MAP_FAILED) { pixels_ = nullptr; error_ = std::string("MMAP: ") + std::strerror(errno); return false; }
        return true;
    }
    ~Framebuffer() {
        if (pixels_) munmap(pixels_, mappingSize_);
        if (fd_ >= 0) close(fd_);
    }
    const std::string& error() const { return error_; }
    int width() const { return static_cast<int>(width_); }
    int height() const { return static_cast<int>(height_); }

    void clear(uint8_t r, uint8_t g, uint8_t b) {
        for (int y = 0; y < height(); ++y) for (int x = 0; x < width(); ++x) pixel(x, y, r, g, b);
    }
    void pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b) {
        if (!pixels_ || x < 0 || y < 0 || x >= width() || y >= height()) return;
        const uint64_t offset = static_cast<uint64_t>(y + var_.yoffset) * fix_.line_length +
            static_cast<uint64_t>(x + var_.xoffset) * (var_.bits_per_pixel / 8);
        const unsigned bytes = var_.bits_per_pixel / 8;
        if (offset + bytes > mappingSize_ || offset + bytes > fix_.smem_len) return;
        const uint32_t packed = channelBits(r, var_.red) | channelBits(g, var_.green) |
                                channelBits(b, var_.blue) | channelBits(255, var_.transp);
        for (unsigned i = 0; i < bytes; ++i) pixels_[offset + i] = static_cast<uint8_t>((packed >> (8 * i)) & 0xff);
    }
    void text(int x, int y, const std::string& value, int scale, uint8_t r, uint8_t g, uint8_t b) {
        int cursor = x;
        for (unsigned char raw : value) {
            const char c = static_cast<char>(raw >= 'a' && raw <= 'z' ? raw - 'a' + 'A' : raw);
            const auto rows = glyph(c);
            for (int row = 0; row < 7; ++row) for (int col = 0; col < 5; ++col) {
                if ((rows[row] & (1u << (4 - col))) == 0) continue;
                for (int sy = 0; sy < scale; ++sy) for (int sx = 0; sx < scale; ++sx)
                    pixel(cursor + col * scale + sx, y + row * scale + sy, r, g, b);
            }
            cursor += 6 * scale;
            if (cursor >= width()) break;
        }
    }
private:
    int fd_ = -1;
    uint8_t* pixels_ = nullptr;
    size_t mappingSize_ = 0;
    fb_fix_screeninfo fix_{};
    fb_var_screeninfo var_{};
    uint32_t width_ = 0, height_ = 0;
    std::string error_;
};

const char* eventTypeName(uint16_t type) {
    switch (type) {
        case EV_KEY: return "KEY"; case EV_REL: return "REL"; case EV_ABS: return "ABS";
        case EV_SW: return "SW"; case EV_MSC: return "MSC"; case EV_LED: return "LED";
        case EV_SND: return "SND"; case EV_REP: return "REP"; case EV_FF: return "FF";
        case EV_PWR: return "PWR"; default: return "OTHER";
    }
}
std::string eventDescription(const std::string& device, const input_event& event) {
    std::ostringstream out;
    out << device << " " << eventTypeName(event.type) << " type=" << event.type
        << " code=" << event.code << " (0x" << std::hex << std::uppercase << event.code
        << std::dec << ") value=" << event.value;
    return out.str();
}

struct InputDevice {
    int fd = -1;
    std::string path, name;
    std::vector<uint8_t> pending;
    InputDevice(int f, std::string p, std::string n) : fd(f), path(std::move(p)), name(std::move(n)) {}
    InputDevice(InputDevice&& other) noexcept
        : fd(other.fd), path(std::move(other.path)), name(std::move(other.name)), pending(std::move(other.pending)) { other.fd = -1; }
    InputDevice& operator=(InputDevice&& other) noexcept {
        if (this != &other) { if (fd >= 0) close(fd); fd = other.fd; other.fd = -1;
            path = std::move(other.path); name = std::move(other.name); pending = std::move(other.pending); }
        return *this;
    }
    InputDevice(const InputDevice&) = delete;
    InputDevice& operator=(const InputDevice&) = delete;
    ~InputDevice() { if (fd >= 0) close(fd); }
};

std::vector<std::string> eventPaths() {
    std::vector<std::string> paths;
    DIR* dir = opendir("/dev/input");
    if (!dir) return paths;
    while (dirent* entry = readdir(dir)) {
        if (std::strncmp(entry->d_name, "event", 5) == 0 && entry->d_name[5] != '\0')
            paths.emplace_back(std::string("/dev/input/") + entry->d_name);
    }
    closedir(dir);
    std::sort(paths.begin(), paths.end());
    return paths;
}

void addLog(std::deque<std::string>& log, const std::string& message) {
    log.push_back(message);
    while (log.size() > 80) log.pop_front();
}

void drawScreen(Framebuffer& fb, const std::deque<std::string>& log, size_t deviceCount, const std::string& status) {
    fb.clear(9, 13, 20);
    const std::string banner = "KOALA MPC READY";
    int scale = std::max(1, std::min(5, (fb.width() - 12) / static_cast<int>(banner.size() * 6)));
    const int bannerWidth = static_cast<int>(banner.size() * 6) * scale - scale;
    fb.text(std::max(4, (fb.width() - bannerWidth) / 2), 8, banner, scale, 80, 255, 170);
    int y = 8 + 9 * scale + 4;
    fb.text(8, y, "MPC SAMPLE INPUT DIAGNOSTIC", 1, 225, 232, 242); y += 12;
    fb.text(8, y, "DEVICES=" + std::to_string(deviceCount) + "  " + status, 1, 100, 190, 255); y += 12;
    fb.text(8, y, "EVENT LOG (TYPE CODE VALUE AND DEVICE NAME)", 1, 255, 205, 90); y += 13;
    for (int x = 0; x < fb.width(); ++x) fb.pixel(x, y - 3, 45, 59, 73);

    const int charsPerLine = std::max(1, (fb.width() - 16) / 6);
    std::vector<std::string> lines;
    for (const auto& entry : log) {
        for (size_t pos = 0; pos < entry.size(); pos += static_cast<size_t>(charsPerLine))
            lines.push_back(entry.substr(pos, static_cast<size_t>(charsPerLine)));
    }
    const int available = std::max(0, (fb.height() - y - 4) / 10);
    const size_t first = lines.size() > static_cast<size_t>(available) ? lines.size() - static_cast<size_t>(available) : 0;
    for (size_t i = first; i < lines.size() && y + 8 < fb.height(); ++i, y += 10)
        fb.text(8, y, lines[i], 1, 210, 220, 232);
}

void scanDevices(std::vector<InputDevice>& devices, std::deque<std::string>& log) {
    for (const auto& path : eventPaths()) {
        const bool present = std::any_of(devices.begin(), devices.end(), [&](const InputDevice& d) { return d.path == path; });
        if (present) continue;
        int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd < 0) {
            addLog(log, path + " OPEN FAILED " + std::strerror(errno));
            std::cerr << path << " OPEN FAILED: " << std::strerror(errno) << '\n';
            continue;
        }
        char name[256] = {};
        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) std::snprintf(name, sizeof(name), "UNKNOWN DEVICE");
        devices.emplace_back(fd, path, name);
        addLog(log, path + " NAME: " + name);
        std::cout << "DEVICE path=" << path << " name=" << name << '\n';
    }
}
} // namespace

int main() {
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    Framebuffer fb;
    if (!fb.openDevice("/dev/fb0")) {
        std::cerr << "Framebuffer unavailable: " << fb.error() << '\n';
        return 1;
    }

    std::deque<std::string> log;
    std::vector<InputDevice> devices;
    addLog(log, "WAITING FOR PAD BUTTON AND ENCODER EVENTS");
    scanDevices(devices, log);
    std::cout << "Framebuffer " << fb.width() << 'x' << fb.height()
              << "; listening for evdev input. Press Ctrl-C to exit.\n";

    int ticksUntilScan = 0;
    drawScreen(fb, log, devices.size(), "LISTENING");
    while (g_running) {
        std::vector<pollfd> fds;
        fds.reserve(devices.size());
        for (const auto& device : devices) fds.push_back({device.fd, POLLIN, 0});
        const int result = poll(fds.data(), fds.size(), 500);
        bool changed = false;
        if (result < 0 && errno != EINTR) {
            addLog(log, std::string("POLL ERROR ") + std::strerror(errno)); changed = true;
        }
        if (result > 0) {
            for (size_t i = devices.size(); i-- > 0;) {
                if (i >= fds.size() || fds[i].revents == 0) continue;
                auto& device = devices[i];
                if (fds[i].revents & (POLLERR | POLLHUP | POLLNVAL)) {
                    addLog(log, device.path + " DISCONNECTED");
                    std::cerr << "DEVICE disconnected path=" << device.path << " name=" << device.name << '\n';
                    devices.erase(devices.begin() + static_cast<std::ptrdiff_t>(i)); changed = true; continue;
                }
                if (!(fds[i].revents & POLLIN)) continue;
                uint8_t buffer[4096];
                const ssize_t count = read(device.fd, buffer, sizeof(buffer));
                if (count < 0) {
                    if (errno != EAGAIN && errno != EINTR) {
                        addLog(log, device.path + " READ ERROR " + std::strerror(errno));
                        devices.erase(devices.begin() + static_cast<std::ptrdiff_t>(i)); changed = true;
                    }
                    continue;
                }
                if (count == 0) continue;
                device.pending.insert(device.pending.end(), buffer, buffer + count);
                size_t consumed = 0;
                while (device.pending.size() - consumed >= sizeof(input_event)) {
                    input_event event{};
                    std::memcpy(&event, device.pending.data() + consumed, sizeof(event));
                    consumed += sizeof(event);
                    if (event.type == EV_SYN) continue;
                    const std::string description = eventDescription(device.name, event);
                    addLog(log, description);
                    std::cout << "EVENT device=\"" << device.name << "\" path=" << device.path
                              << " type=" << event.type << " (" << eventTypeName(event.type) << ")"
                              << " code=" << event.code << " (0x" << std::hex << std::uppercase
                              << event.code << std::dec << ") value=" << event.value << '\n';
                    changed = true;
                }
                if (consumed) device.pending.erase(device.pending.begin(), device.pending.begin() + static_cast<std::ptrdiff_t>(consumed));
            }
        }
        if (++ticksUntilScan >= 4) {
            ticksUntilScan = 0;
            const size_t oldSize = devices.size();
            scanDevices(devices, log);
            if (oldSize != devices.size()) changed = true;
        }
        if (changed) drawScreen(fb, log, devices.size(), "LISTENING");
    }
    addLog(log, "DIAGNOSTIC STOPPED");
    drawScreen(fb, log, devices.size(), "STOPPED");
    std::cout << "Diagnostic stopped.\n";
    return 0;
}
