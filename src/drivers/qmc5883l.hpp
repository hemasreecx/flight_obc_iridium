#ifndef QMC5883L_HPP
#define QMC5883L_HPP

#include <stdint.h>
#include <stddef.h>
#include "hardware/i2c.h"

#ifndef QMC5883L_DEBUG
#define QMC5883L_DEBUG 0
#endif

#define QMC5883L_ADDR 0x0D  // fixed

enum class QMC5883L_Register : uint8_t {
    X_LSB     = 0x00,
    X_MSB     = 0x01,
    Y_LSB     = 0x02,
    Y_MSB     = 0x03,
    Z_LSB     = 0x04,
    Z_MSB     = 0x05,
    STATUS    = 0x06,
    TEMP_LSB  = 0x07,
    TEMP_MSB  = 0x08,
    CONTROL_1 = 0x09,
    CONTROL_2 = 0x0A,
    SET_RESET = 0x0B
};

constexpr uint8_t QMC5883L_STATUS_DRDY = (1 << 0); // Data Ready
constexpr uint8_t QMC5883L_STATUS_OVL  = (1 << 1); // Overflow
constexpr uint8_t QMC5883L_STATUS_DOR  = (1 << 2); // Data Skipped

enum class QMC5883L_Range : uint8_t {
    RANGE_2G = 0b00,
    RANGE_8G = 0b01
};

enum class QMC5883L_ODR : uint8_t {
    ODR_10HZ  = 0b00,
    ODR_50HZ  = 0b01,
    ODR_100HZ = 0b10,
    ODR_200HZ = 0b11
};

enum class QMC5883L_Mode : uint8_t {
    STANDBY    = 0b00,
    CONTINUOUS = 0b01
};

enum class QMC5883L_OSR : uint8_t {
    OSR_512 = 0b00,
    OSR_256 = 0b01,
    OSR_128 = 0b10,
    OSR_64  = 0b11
};

enum class QMC5883L_Status : uint8_t {
    OK = 0,
    NOT_READY,        // isDataReady() returned false
    READ_ERROR,       // I2C read failed
    ERR_COMM,         // general communication error
    ERR_INVALID_ID,   // unexpected chip ID
    ERR_CONFIG        // configure() failed
};

struct QMC5883L_Raw {
    int16_t x;
    int16_t y;
    int16_t z;
};

struct QMC5883L_Config {
    QMC5883L_Range range;
    QMC5883L_ODR   odr;
    QMC5883L_Mode  mode;
    QMC5883L_OSR   osr;
    bool enable_drdy_interrupt;
    bool pointer_roll_over;
};

class QMC5883L {
public:
    QMC5883L(i2c_inst_t* i2c_port, uint8_t address = QMC5883L_ADDR);

    bool init();
    bool configure(const QMC5883L_Config& config);
    bool soft_reset();   // fixed name

    bool readRaw(QMC5883L_Raw& raw_data);
    bool readTemperature(float& temperature);

    bool isDataReady();
    bool hasOverflow();
    bool hasDataSkipped();

    bool standby();

private:
    i2c_inst_t* i2c_port_;
    uint8_t     address_;

    bool write_register(QMC5883L_Register reg, uint8_t value);
    bool read_register(QMC5883L_Register reg, uint8_t& value);
    bool read_multiple_registers(QMC5883L_Register start_reg,
                                  uint8_t* buffer,
                                  size_t length);
    static int16_t combine(uint8_t low, uint8_t high);
};

#endif 
