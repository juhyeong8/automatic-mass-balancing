#include "pololu_tic_i2c.hpp"

bool TicI2C::energize(I2CBus& bus) const {
    const uint8_t cmd[] = { 0x85 };
    return bus.write(m_addr7, cmd, sizeof(cmd));
}

bool TicI2C::exitSafeStart(I2CBus& bus) const {
    const uint8_t cmd[] = { 0x83 };
    return bus.write(m_addr7, cmd, sizeof(cmd));
}

bool TicI2C::resetCommandTimeout(I2CBus& bus) const {
    const uint8_t cmd[] = { 0x8C };
    return bus.write(m_addr7, cmd, sizeof(cmd));
}

bool TicI2C::setTargetPosition(I2CBus& bus, int32_t target) const {
    uint8_t cmd[] = {
        0xE0,
        static_cast<uint8_t>((target >> 0)  & 0xFF),
        static_cast<uint8_t>((target >> 8)  & 0xFF),
        static_cast<uint8_t>((target >> 16) & 0xFF),
        static_cast<uint8_t>((target >> 24) & 0xFF),
    };
    return bus.write(m_addr7, cmd, sizeof(cmd));
}

bool TicI2C::haltAndSetPosition(I2CBus& bus, int32_t position) const {
    uint8_t cmd[] = {
        0xEC,
        static_cast<uint8_t>((position >> 0)  & 0xFF),
        static_cast<uint8_t>((position >> 8)  & 0xFF),
        static_cast<uint8_t>((position >> 16) & 0xFF),
        static_cast<uint8_t>((position >> 24) & 0xFF),
    };
    return bus.write(m_addr7, cmd, sizeof(cmd));
}
