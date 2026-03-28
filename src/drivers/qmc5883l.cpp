#include "qmc5883l.hpp"
#include "pico/stdlib.h"
#include "pico/timeout_helper.h"
#include <stdio.h>

#if QMC5883L_DEBUG
#define QMC5883L_LOG(...) printf(__VA_ARGS__)
#else
#define QMC5883L_LOG(...)
#endif

constexpr int QMC5883L_MAX_RETRIES = 3;
constexpr uint32_t QMC5883L_TIMEOUT_MS = 1000;

QMC5883L::QMC5883L(i2c_inst_t* i2c_port, uint8_t address)
    : i2c_port_(i2c_port), address_(address) {}

int16_t QMC5883L::combine(uint8_t low, uint8_t high) {
    return (static_cast<int16_t>(high) << 8) | static_cast<int16_t>(low);
}

bool QMC5883L::write_register(QMC5883L_Register reg, uint8_t value) {
    uint8_t buffer[2] = {static_cast<uint8_t>(reg), value};
    for (int attempt = 0; attempt < QMC5883L_MAX_RETRIES; ++attempt) {
        int ret = i2c_write_blocking_until(
            i2c_port_, address_, buffer, sizeof(buffer), false,
            make_timeout_time_ms(QMC5883L_TIMEOUT_MS)
        );
        if (ret == sizeof(buffer)) {
            return true;
        }
        sleep_us(100);
        QMC5883L_LOG("Write attempt %d failed: %d\n", attempt + 1, ret);
    }
    return false;
}

bool QMC5883L::read_register(QMC5883L_Register reg, uint8_t& value) {
    uint8_t reg_addr = static_cast<uint8_t>(reg);
    for (int attempt = 0; attempt < QMC5883L_MAX_RETRIES; ++attempt) {
        int ret = i2c_write_blocking_until(
            i2c_port_, address_, &reg_addr, 1, true,
            make_timeout_time_ms(QMC5883L_TIMEOUT_MS)
        );
        if (ret != 1) {
            sleep_us(100);
            QMC5883L_LOG("Read reg write phase attempt %d failed: %d\n", attempt + 1, ret);
            continue;
        }
        ret = i2c_read_blocking_until(
            i2c_port_, address_, &value, 1, false,
            make_timeout_time_ms(QMC5883L_TIMEOUT_MS)
        );
        if (ret == 1) {
            return true;
        }
        sleep_us(100);
        QMC5883L_LOG("Read reg read phase attempt %d failed: %d\n", attempt + 1, ret);
    }
    return false;
}

bool QMC5883L::read_multiple_registers(QMC5883L_Register start_reg, uint8_t* buffer, size_t length) {
    uint8_t reg_addr = static_cast<uint8_t>(start_reg);
    for (int attempt = 0; attempt < QMC5883L_MAX_RETRIES; ++attempt) {
        int ret = i2c_write_blocking_until(
            i2c_port_, address_, &reg_addr, 1, true,  // true = repeated start
            make_timeout_time_ms(QMC5883L_TIMEOUT_MS)
        );
        if (ret != 1) {
            sleep_us(100);
            QMC5883L_LOG("Read multiple write phase attempt %d failed: %d\n", attempt + 1, ret);
            continue;  
        }
        ret = i2c_read_blocking_until(
            i2c_port_, address_, buffer, length, false,
            make_timeout_time_ms(QMC5883L_TIMEOUT_MS)
        );
        if (ret == static_cast<int>(length)) {
            return true;
        }
        sleep_us(100);
        QMC5883L_LOG("Read multiple read phase attempt %d failed: %d\n", attempt + 1, ret);
    }
    return false;
}

bool QMC5883L::soft_reset() {
    if (!write_register(QMC5883L_Register::CONTROL_2, 0x80)) {
        QMC5883L_LOG("Soft reset failed\n");
        return false;
    }
    sleep_ms(10); 
    return true;
}
/// is there any self test kinda thing if we can add before going to the main loop
bool QMC5883L::init() { // require to add more feature check in the init method like  - range set , ODR, OSR, ETC..
    if (!soft_reset()) {
        QMC5883L_LOG("Init failed at soft reset\n");
        return false;
    }

    if (!write_register(QMC5883L_Register::SET_RESET, 0x01)) {
        QMC5883L_LOG("Init failed at SET/RESET period\n");
        return false;
    }

    QMC5883L_LOG("QMC5883L init success\n");
    return true;
}

bool QMC5883L::configure(const QMC5883L_Config& config) {
    // Bits: [7:6] OSR | [5:4] RNG | [3:2] ODR | [1:0] MODE
    uint8_t ctrl1 = 0;
    ctrl1 |= (static_cast<uint8_t>(config.osr)  << 6);
    ctrl1 |= (static_cast<uint8_t>(config.range) << 4);
    ctrl1 |= (static_cast<uint8_t>(config.odr)   << 2);
    ctrl1 |= (static_cast<uint8_t>(config.mode)  << 0);

    if (!write_register(QMC5883L_Register::CONTROL_1, ctrl1)) {
        QMC5883L_LOG("Configure failed at CONTROL_1\n");
        return false;
    }
    uint8_t ctrl2 = 0;
    ctrl2 |= (config.pointer_roll_over  ? (1 << 6) : 0);
    ctrl2 |= (config.enable_drdy_interrupt ? 0 : (1 << 0)); // inverted!

    if (!write_register(QMC5883L_Register::CONTROL_2, ctrl2)) {
        QMC5883L_LOG("Configure failed at CONTROL_2\n");
        return false;
    }

    QMC5883L_LOG("QMC5883L configure success\n");
    return true;
}

bool QMC5883L::readRaw(QMC5883L_Raw& raw_data) {
    uint8_t buffer[6];
    if (!read_multiple_registers(QMC5883L_Register::X_LSB, buffer, sizeof(buffer))) {
        QMC5883L_LOG("readRaw failed\n");
        return false;
    }
    raw_data.x = combine(buffer[0], buffer[1]);
    raw_data.y = combine(buffer[2], buffer[3]);
    raw_data.z = combine(buffer[4], buffer[5]);
    return true;
}

bool QMC5883L::readTemperature(float& temperature) {
    uint8_t temp_lsb, temp_msb;
    if (!read_register(QMC5883L_Register::TEMP_LSB, temp_lsb) ||
        !read_register(QMC5883L_Register::TEMP_MSB, temp_msb)) {
        QMC5883L_LOG("readTemperature failed\n");
        return false;
    }
    int16_t temp_raw = combine(temp_lsb, temp_msb);
    temperature = static_cast<float>(temp_raw) / 100.0f;
    return true;
}

bool QMC5883L::isDataReady() {
    uint8_t status;
    if (!read_register(QMC5883L_Register::STATUS, status)) {
        return false;
    }
    return (status & QMC5883L_STATUS_DRDY) != 0;
}

bool QMC5883L::hasOverflow() {
    uint8_t status;
    if (!read_register(QMC5883L_Register::STATUS, status)) {
        return false;
    }
    return (status & QMC5883L_STATUS_OVL) != 0;
}

bool QMC5883L::hasDataSkipped() {
    uint8_t status;
    if (!read_register(QMC5883L_Register::STATUS, status)) {
        return false;
    }
    return (status & QMC5883L_STATUS_DOR) != 0;
}

bool QMC5883L::standby() {
    uint8_t control_value;
    // read current CONTROL_1 to preserve OSR/RNG/ODR bits
    if (!read_register(QMC5883L_Register::CONTROL_1, control_value)) {
        QMC5883L_LOG("standby read failed\n");
        return false;
    }
    control_value &= ~(0b11); // clear mode bits [1:0]
    control_value |= static_cast<uint8_t>(QMC5883L_Mode::STANDBY);
    return write_register(QMC5883L_Register::CONTROL_1, control_value);
}