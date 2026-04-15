#include "ina260.hpp"

Ina260::Ina260(i2c_inst_t* instance, uint8_t device_addr, uint32_t timeout_us)
    : _instance(instance),
      _device_addr(device_addr),
      _timeout_us(timeout_us),
      _config_cache(0)
{
}

bool Ina260::writeRegister(Register reg, uint16_t value) {
    if (_instance == nullptr) return false;

    uint8_t buffer[3];
    buffer[0] = static_cast<uint8_t>(reg);
    buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buffer[2] = static_cast<uint8_t>(value & 0xFF);

    for (int attempt = 0; attempt < INA260_MAX_RETRIES; attempt++) {
        int written = i2c_write_blocking_until(_instance, _device_addr, buffer, 3, false, make_timeout_time_us(_timeout_us));
        
        if (written == 3) {
            if (reg == Register::CONFIG) _config_cache = value;
            return true;
        }
        sleep_us(100); // Wait before retry
    }
    return false;
}

bool Ina260::readRegister(Register reg, uint16_t& value) {
    if (_instance == nullptr) return false;
    uint8_t reg_addr = static_cast<uint8_t>(reg);

    for (int attempt = 0; attempt < INA260_MAX_RETRIES; attempt++) {
        // Write address with repeated start (nostop = true)
        int written = i2c_write_blocking_until(_instance, _device_addr, &reg_addr, 1, true, make_timeout_time_us(_timeout_us));
        
        if (written != 1) {
            // Write failed — force a STOP condition to clean up the bus
            i2c_write_blocking_until(_instance, _device_addr, &reg_addr, 1, false, make_timeout_time_us(_timeout_us));
            sleep_us(100);
            continue;
        }

        uint8_t buffer[2];
        int read_bytes = i2c_read_blocking_until(_instance, _device_addr, buffer, 2, false, make_timeout_time_us(_timeout_us));
        
        if (read_bytes == 2) {
            value = static_cast<uint16_t>((buffer[0] << 8) | buffer[1]);
            return true;
        }
        sleep_us(100);
    }
    return false;
}

bool Ina260::updateConfig(uint16_t mask, uint16_t value) {
    uint16_t cfg = 0;
    if (!readRegister(Register::CONFIG, cfg)) return false;
    cfg = static_cast<uint16_t>((cfg & ~mask) | (value & mask));
    return writeRegister(Register::CONFIG, cfg);
}

bool Ina260::init() {
    // Config: continuous bus + shunt, 4x averaging
    return writeRegister(Register::CONFIG, 0x6127);
}

bool Ina260::reset() {
    return writeRegister(Register::CONFIG, 0x8000);
}

bool Ina260::setMode(Mode mode) {
    return updateConfig(0x0007, static_cast<uint16_t>(mode));
}

bool Ina260::setConversionTime(ConversionTime v_time, ConversionTime c_time) {
    uint16_t v = static_cast<uint16_t>(static_cast<uint16_t>(v_time) << 6);
    uint16_t c = static_cast<uint16_t>(static_cast<uint16_t>(c_time) << 3);
    constexpr uint16_t kMask = static_cast<uint16_t>((0x7u << 6) | (0x7u << 3));
    return updateConfig(kMask, static_cast<uint16_t>(v | c));
}

bool Ina260::setAveraging(Averaging count) {
    return updateConfig(0x0E00, static_cast<uint16_t>(count));
}

bool Ina260::readCurrent(float& current_ma) {
    uint16_t raw_val = 0;
    if (!readRegister(Register::CURRENT, raw_val)) return false;

    int16_t signed_val = static_cast<int16_t>(raw_val);
    current_ma = static_cast<float>(signed_val) * 1.25f;
    return true;
}

bool Ina260::readVoltage(float& voltage_mv) {
    uint16_t raw_val = 0;
    if (!readRegister(Register::VOLTAGE, raw_val)) return false;

    voltage_mv = static_cast<float>(raw_val) * 1.25f;
    return true;
}

bool Ina260::readAll(float& current_ma, float& voltage_mv) {
    return readCurrent(current_ma) && readVoltage(voltage_mv);
}