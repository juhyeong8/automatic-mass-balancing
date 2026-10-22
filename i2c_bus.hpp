#pragma once
#include <cstdint>
#include <string>

class I2CBus {
public:
    I2CBus() = default;
    explicit I2CBus(const std::string& dev);

    ~I2CBus();

    I2CBus(const I2CBus&) = delete;
    I2CBus& operator=(const I2CBus&) = delete;

    I2CBus(I2CBus&& other) noexcept;
    I2CBus& operator=(I2CBus&& other) noexcept;

    bool open(const std::string& dev);
    void close();

    int fd() const { return m_fd; }
    bool valid() const { return m_fd >= 0; }

    // Single-message write using I2C_RDWR (recommended)
    bool write(uint8_t addr7, const uint8_t* data, uint16_t len);

    // Optional read (if you need it later)
    bool read(uint8_t addr7, uint8_t* data, uint16_t len);

    // Simple ACK probe (sets I2C_SLAVE then does 0-len write)
    bool probeAck(uint8_t addr7);

    const std::string& devicePath() const { return m_dev; }

private:
    int m_fd{-1};
    std::string m_dev;
};
