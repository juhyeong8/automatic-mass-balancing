#include "tca9548a.hpp"
#include <cerrno>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

bool TCA9548A::select(I2CBus& bus, uint8_t channel) const {
    if (!bus.valid()) return false;
    if (channel > 7) {
        std::cerr << "[MUX] invalid channel: " << int(channel) << "\n";
        return false;
    }

    // For mux, I2C_SLAVE + write 1 byte is simplest/robust
    if (::ioctl(bus.fd(), I2C_SLAVE, m_addr7) < 0) {
        std::cerr << "[MUX] ioctl(I2C_SLAVE) failed: " << std::strerror(errno) << "\n";
        return false;
    }

    uint8_t mask = static_cast<uint8_t>(1u << channel);
    if (::write(bus.fd(), &mask, 1) != 1) {
        std::cerr << "[MUX] channel select write failed: " << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}
