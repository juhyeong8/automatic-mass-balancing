#include "i2c_bus.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

I2CBus::I2CBus(const std::string& dev) { open(dev); }

I2CBus::~I2CBus() { close(); }

I2CBus::I2CBus(I2CBus&& other) noexcept {
    m_fd = other.m_fd;
    m_dev = std::move(other.m_dev);
    other.m_fd = -1;
}

I2CBus& I2CBus::operator=(I2CBus&& other) noexcept {
    if (this != &other) {
        close();
        m_fd = other.m_fd;
        m_dev = std::move(other.m_dev);
        other.m_fd = -1;
    }
    return *this;
}

bool I2CBus::open(const std::string& dev) {
    close();
    m_dev = dev;
    m_fd = ::open(dev.c_str(), O_RDWR);
    if (m_fd < 0) {
        std::cerr << "[I2C] open failed: " << dev << " : " << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

void I2CBus::close() {
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool I2CBus::write(uint8_t addr7, const uint8_t* data, uint16_t len) {
    if (m_fd < 0) return false;

    i2c_msg msg{};
    msg.addr  = addr7;
    msg.flags = 0;
    msg.len   = len;
    msg.buf   = const_cast<uint8_t*>(data);

    i2c_rdwr_ioctl_data xfer{};
    xfer.msgs  = &msg;
    xfer.nmsgs = 1;

    int r = ::ioctl(m_fd, I2C_RDWR, &xfer);
    if (r != 1) {
        std::cerr << "[I2C] write(I2C_RDWR) failed addr7=" << int(addr7)
                  << " : " << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

bool I2CBus::read(uint8_t addr7, uint8_t* data, uint16_t len) {
    if (m_fd < 0) return false;

    i2c_msg msg{};
    msg.addr  = addr7;
    msg.flags = I2C_M_RD;
    msg.len   = len;
    msg.buf   = data;

    i2c_rdwr_ioctl_data xfer{};
    xfer.msgs  = &msg;
    xfer.nmsgs = 1;

    int r = ::ioctl(m_fd, I2C_RDWR, &xfer);
    if (r != 1) {
        std::cerr << "[I2C] read(I2C_RDWR) failed addr7=" << int(addr7)
                  << " : " << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

bool I2CBus::probeAck(uint8_t addr7) {
    if (m_fd < 0) return false;

    if (::ioctl(m_fd, I2C_SLAVE, addr7) < 0) {
        return false;
    }

    // zero-length write as an ACK probe
    int r = ::write(m_fd, nullptr, 0);
    if (r < 0 && errno == EREMOTEIO) return false;
    return true;
}
