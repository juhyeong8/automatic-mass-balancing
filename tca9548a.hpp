#pragma once
#include <cstdint>
#include "i2c_bus.hpp"

class TCA9548A {
public:
    explicit TCA9548A(uint8_t addr7) : m_addr7(addr7) {}

    uint8_t addr7() const { return m_addr7; }

    // Select exactly one channel (0..7)
    bool select(I2CBus& bus, uint8_t channel) const;

private:
    uint8_t m_addr7;
};
