#ifndef INA260_HPP
#define INA260_HPP

#include <stdint.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#ifdef DEBUG_INA260
    #include <stdio.h>
    #define INA_LOG(fmt, ...) printf("[INA260] " fmt "\n", ##__VA_ARGS__)
#else
    #define INA_LOG(fmt, ...) ((void)0)
#endif

constexpr int INA260_MAX_RETRIES = 3;

class Ina260 {
public:
    static constexpr uint8_t DEFAULT_ADDR = 0x40;
    static constexpr uint32_t DEFAULT_TIMEOUT_US = 1000; // 1ms per operation

    enum class Register : uint8_t {
        CONFIG           = 0x00,
        CURRENT          = 0x01,
        VOLTAGE          = 0x02,
        POWER            = 0x03,
        MASK_ENABLE      = 0x06,
        ALERT_LIMIT      = 0x07,
        MANUFACTURER_ID  = 0xFE,
        DIE_ID           = 0xFF
    };

    enum class Averaging : uint16_t {
        COUNT_1    = 0x000,
        COUNT_4    = 0x200,
        COUNT_16   = 0x400,
        COUNT_64   = 0x600,
        COUNT_128  = 0x800,
        COUNT_256  = 0xA00,
        COUNT_512  = 0xC00,
        COUNT_1024 = 0xE00
    };

    enum class ConversionTime : uint16_t {
        TIME_140us  = 0x00,
        TIME_204us  = 0x01,
        TIME_332us  = 0x02,
        TIME_588us  = 0x03,
        TIME_1ms    = 0x04, 
        TIME_2ms    = 0x05, 
        TIME_4ms    = 0x06, 
        TIME_8ms    = 0x07  
    };

    enum class Mode : uint16_t {
        POWER_DOWN          = 0x00,
        TRIGGERED_CURRENT   = 0x01,
        TRIGGERED_VOLTAGE   = 0x02,
        TRIGGERED_ALL       = 0x03,
        CONTINUOUS_CURRENT  = 0x05,
        CONTINUOUS_VOLTAGE  = 0x06,
        CONTINUOUS_ALL      = 0x07
    };

    Ina260(i2c_inst_t* instance, uint8_t device_addr = DEFAULT_ADDR, uint32_t timeout_us = DEFAULT_TIMEOUT_US);

    bool init();
    bool reset();

    bool setMode(Mode mode);
    bool setConversionTime(ConversionTime v_time, ConversionTime c_time);
    bool setAveraging(Averaging count);

    bool readCurrent(float& current_ma);
    bool readVoltage(float& voltage_mv);
    bool readAll(float& current_ma, float& voltage_mv);

private:
    i2c_inst_t* _instance;
    uint8_t     _device_addr;
    uint32_t    _timeout_us;
    uint16_t    _config_cache;

    bool writeRegister(Register reg, uint16_t value);
    bool readRegister(Register reg, uint16_t& value);
    bool updateConfig(uint16_t mask, uint16_t value);
};

#endif