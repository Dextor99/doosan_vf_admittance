#pragma once
// Read-only Modbus RTU reader derived from doosan_cpp/xjc_force.cpp.
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <string>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

namespace {

using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t stop_requested = 0;

void on_signal(int) {
    stop_requested = 1;
}

uint16_t modbus_crc(const uint8_t* data, std::size_t length) {
    uint16_t crc = 0xffff;
    for (std::size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = static_cast<uint16_t>((crc >> 1) ^ ((crc & 1) ? 0xa001 : 0));
        }
    }
    return crc;
}

std::string system_error(const std::string& action) {
    return action + ": " + std::strerror(errno);
}

class SerialPort {
public:
    SerialPort() : fd_(-1), configured_(false) {}
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    ~SerialPort() {
        if (fd_ >= 0) {
            if (configured_ && tcsetattr(fd_, TCSANOW, &original_) != 0) {
                std::cerr << system_error("Could not restore serial settings") << '\n';
            }
            close(fd_);
        }
    }

    bool connect(const std::string& path, std::string& error) {
        fd_ = open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) {
            error = system_error("Cannot open " + path);
            return false;
        }
        if (flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            error = system_error("Cannot lock " + path + " (another reader may be using it)");
            return false;
        }
        if (tcgetattr(fd_, &original_) != 0) {
            error = system_error("Cannot read serial settings");
            return false;
        }
        termios config = original_;
        cfmakeraw(&config);
        config.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
        config.c_cflag |= CS8 | CREAD | CLOCAL;
        config.c_cc[VMIN] = 0;
        config.c_cc[VTIME] = 0;
        if (cfsetispeed(&config, B115200) != 0 || cfsetospeed(&config, B115200) != 0) {
            error = system_error("Cannot set baud rate");
            return false;
        }
        if (tcsetattr(fd_, TCSANOW, &config) != 0) {
            error = system_error("Cannot apply serial settings");
            return false;
        }
        configured_ = true;
        return true;
    }

    int fd() const { return fd_; }

private:
    int fd_;
    termios original_{};
    bool configured_;
};

// One FC04 request reads 12 registers starting at address 0 from station 1.
bool read_frame(int fd, std::array<uint8_t, 29>& frame, std::string& error) {
    std::array<uint8_t, 8> request{{0x01, 0x04, 0x00, 0x00, 0x00, 0x0c, 0, 0}};
    const uint16_t request_crc = modbus_crc(request.data(), 6);
    request[6] = static_cast<uint8_t>(request_crc & 0xff);
    request[7] = static_cast<uint8_t>(request_crc >> 8);

    if (tcflush(fd, TCIFLUSH) != 0) {
        error = system_error("Cannot clear old input");
        return false;
    }
    const auto deadline = Clock::now() + std::chrono::milliseconds(500);
    std::size_t sent = 0;
    std::size_t received = 0;
    std::size_t expected = frame.size();

    while (!stop_requested && Clock::now() < deadline && sent < request.size()) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        if (ms <= 0) break;
        pollfd entry{fd, POLLOUT, 0};
        const int ready = poll(&entry, 1, static_cast<int>(ms));
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0) { error = system_error("Serial write poll failed"); return false; }
        if (ready == 0) break;
        if (entry.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            error = "Serial device disconnected during request";
            return false;
        }
        const ssize_t count = write(fd, request.data() + sent, request.size() - sent);
        if (count > 0) sent += static_cast<std::size_t>(count);
        else if (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
            error = system_error("Serial write failed");
            return false;
        }
    }
    if (sent != request.size()) {
        error = "Timed out while sending Modbus request";
        return false;
    }

    while (!stop_requested && Clock::now() < deadline && received < expected) {
        const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        if (ms <= 0) break;
        pollfd entry{fd, POLLIN, 0};
        const int ready = poll(&entry, 1, static_cast<int>(ms));
        if (ready < 0 && errno == EINTR) continue;
        if (ready < 0) { error = system_error("Serial read poll failed"); return false; }
        if (ready == 0) break;
        if (entry.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            error = "Serial device disconnected while awaiting reply";
            return false;
        }
        const ssize_t count = read(fd, frame.data() + received, expected - received);
        if (count > 0) received += static_cast<std::size_t>(count);
        else if (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
            error = system_error("Serial read failed");
            return false;
        }
        if (received >= 2 && frame[1] == 0x84) expected = 5;
        if (received >= 3 && frame[1] == 0x04 && frame[2] != 24) {
            error = "Unexpected Modbus data length";
            return false;
        }
    }

    if (received != expected) {
        error = "Timed out waiting for a complete Modbus response";
        return false;
    }
    const uint16_t actual_crc = static_cast<uint16_t>(frame[expected - 2]) |
                                (static_cast<uint16_t>(frame[expected - 1]) << 8);
    if (modbus_crc(frame.data(), expected - 2) != actual_crc) {
        error = "Modbus CRC mismatch";
        return false;
    }
    if (frame[0] != 1) {
        error = "Response has an unexpected station address";
        return false;
    }
    if (frame[1] == 0x84) {
        error = "Modbus exception code " + std::to_string(frame[2]);
        return false;
    }
    if (frame[1] != 0x04 || frame[2] != 24) {
        error = "Unexpected Modbus function or data length";
        return false;
    }
    return true;
}

bool decode_values(const std::array<uint8_t, 29>& frame, std::array<float, 6>& values) {
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
                  "This program requires IEEE 754 binary32 floats");
    for (std::size_t i = 0; i < values.size(); ++i) {
        const std::size_t p = 3 + i * 4;
        const uint32_t bits = (static_cast<uint32_t>(frame[p]) << 24) |
                              (static_cast<uint32_t>(frame[p + 1]) << 16) |
                              (static_cast<uint32_t>(frame[p + 2]) << 8) |
                              static_cast<uint32_t>(frame[p + 3]);
        std::memcpy(&values[i], &bits, sizeof(bits));
        if (!std::isfinite(values[i])) return false;
    }
    return true;
}

}  // namespace
