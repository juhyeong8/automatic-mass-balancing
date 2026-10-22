#pragma once
#include <cstdint>
#include "i2c_bus.hpp"

class TicI2C {
public:
    explicit TicI2C(uint8_t addr7) : m_addr7(addr7) {}

    uint8_t addr7() const { return m_addr7; }

    bool energize(I2CBus& bus) const;
    bool exitSafeStart(I2CBus& bus) const;
    bool resetCommandTimeout(I2CBus& bus) const;

    bool setTargetPosition(I2CBus& bus, int32_t target) const;
    bool haltAndSetPosition(I2CBus& bus, int32_t position) const;

private:
    uint8_t m_addr7;
};
