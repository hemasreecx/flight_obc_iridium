#include "ls_acquisition.hpp"
#include "pico/stdlib.h"
#include "pico/time.h"

// ─── Constructor ──────────────────────────────────────────────────────────────

LSM6DSV80X_Acquisition::LSM6DSV80X_Acquisition(LSM6DSV80X& imu, uint8_t rate_hz)
    : _imu(imu)
    , _rate_hz(rate_hz)
    , _last_sample_time_us(0)
    , _converter(LSM6DSV80X_GY_Range::RANGE_1000DPS,
                 LSM6DSV80X_HG_Range::RANGE_80G)
    , _gy_filter()
    , _hg_filter()
    , _raw_gy{}
    , _raw_hg{}
    , _raw_temp(0)
    , _temp_offset(0)
    , _filtered_gy{}
    , _filtered_hg{}
{
}

// ─── init ─────────────────────────────────────────────────────────────────────

bool LSM6DSV80X_Acquisition::init()
{
    LSM6DSV80X_Config config{};

    // ── Gyroscope ─────────────────────────────────────────────────────────────
    config.gy_range = LSM6DSV80X_GY_Range::RANGE_1000DPS;
    config.gy_odr   = odrForHz(_rate_hz);
    config.gy_mode  = LSM6DSV80X_GY_Mode::HIGH_PERFORMANCE;

    // ── High-G accelerometer ──────────────────────────────────────────────────
    // HG ODR minimum is 480Hz — always fresher than our poll rate.
    config.enable_hg_xl = true;
    config.hg_range     = LSM6DSV80X_HG_Range::RANGE_80G;
    config.hg_odr       = LSM6DSV80X_HG_ODR::HG_ODR_480HZ;
    config.hg_mode      = LSM6DSV80X_HG_Mode::HIGH_PERFORMANCE;

    // ── Low-G accelerometer ────────────────────────────────────────────────────
    // Keep XL clock domain active; HG data path depends on it on this device.
    config.xl_range = LSM6DSV80X_XL_Range::RANGE_4G;
    config.xl_odr   = LSM6DSV80X_ODR::ODR_120HZ;
    config.xl_mode  = LSM6DSV80X_XL_Mode::HIGH_PERFORMANCE;

    // ── Common ────────────────────────────────────────────────────────────────
    config.enable_bdu        = true;
    config.enable_timestamp  = false;
    config.enable_drdy_int1  = false;
    config.enable_fifo       = false;
    config.fifo_mode         = LSM6DSV80X_FIFO_Mode::BYPASS;
    config.fifo_watermark    = 0;

    if (_imu.init(config) != LSM6DSV80X_Status::OK)
        return false;

    _last_sample_time_us = to_us_since_boot(get_absolute_time());
    return true;
}

// ─── task ─────────────────────────────────────────────────────────────────────
//
// No dataReady() checks — all three sensors are read directly every tick.
//
// Why this is safe:
//   GY   ODR >= poll rate  → register always has fresh data
//   HG   ODR = 480Hz       → always fresher than our poll rate
//   TEMP updates ~1Hz internally but reading more often just returns
//          the same value — no harm done

LSM6DSV80X_Status LSM6DSV80X_Acquisition::task()
{
    if (_rate_hz == 0)
        return LSM6DSV80X_Status::ERR_CONFIG;

    // uint32_t subtraction wraps correctly on rollover — intentional.
    uint32_t now_us    = to_us_since_boot(get_absolute_time());
    uint32_t period_us = 1000000U / _rate_hz;

    if ((now_us - _last_sample_time_us) < period_us)
        return LSM6DSV80X_Status::SKIPPED;   // period not elapsed — nothing to do

    _last_sample_time_us = now_us;

    // ── Gyroscope ─────────────────────────────────────────────────────────────
    {
        LSM6DSV80X_Status s = _imu.readRawGY(_raw_gy);
        if (s != LSM6DSV80X_Status::OK) return s;

        LSM6DSV80X_Data converted = _converter.convertGY(_raw_gy);
        _filtered_gy = _gy_filter.update(converted);
    }

    // ── High-G accelerometer ──────────────────────────────────────────────────
    {
        LSM6DSV80X_Status s = _imu.readRawHG(_raw_hg);
        if (s != LSM6DSV80X_Status::OK) return s;

        LSM6DSV80X_Data converted = _converter.convertHG(_raw_hg);
        _filtered_hg = _hg_filter.update(converted);
    }

    // ── Temperature ───────────────────────────────────────────────────────────
    {
        int16_t raw = 0;
        LSM6DSV80X_Status s = _imu.readTemperature(raw);
        if (s != LSM6DSV80X_Status::OK) return s;

        _raw_temp = raw - _temp_offset;
    }

    return LSM6DSV80X_Status::OK;
}

// ─── setRateHz ────────────────────────────────────────────────────────────────

void LSM6DSV80X_Acquisition::setRateHz(uint8_t hz)
{
    if (hz == 0)   hz = 1;
    if (hz > 240)  hz = 240;

    _rate_hz = hz;
    _last_sample_time_us = to_us_since_boot(get_absolute_time());
    reconfigureODR(hz);
}

// ─── GY / HG offset setters ───────────────────────────────────────────────────

void LSM6DSV80X_Acquisition::setGYOffsets(int16_t x, int16_t y, int16_t z)
{
    _converter.setGYOffsets(x, y, z);
}

void LSM6DSV80X_Acquisition::setHGOffsets(int16_t x, int16_t y, int16_t z)
{
    _converter.setHGOffsets(x, y, z);
}

void LSM6DSV80X_Acquisition::clearGYOffsets()
{
    _converter.clearGYOffsets();
}

void LSM6DSV80X_Acquisition::clearHGOffsets()
{
    _converter.clearHGOffsets();
}

// ─── Temperature offset setters ───────────────────────────────────────────────

void LSM6DSV80X_Acquisition::setTempOffset(int16_t offset)
{
    _temp_offset = offset;
}

void LSM6DSV80X_Acquisition::clearTempOffset()
{
    _temp_offset = 0;
}

// ─── resetFilters ─────────────────────────────────────────────────────────────

void LSM6DSV80X_Acquisition::resetFilters()
{
    _gy_filter.reset();
    _hg_filter.reset();
}

// ─── reconfigureODR ───────────────────────────────────────────────────────────

bool LSM6DSV80X_Acquisition::reconfigureODR(uint8_t hz)
{
    LSM6DSV80X_Config config{};

    config.gy_range = LSM6DSV80X_GY_Range::RANGE_1000DPS;
    config.gy_odr   = odrForHz(hz);
    config.gy_mode  = LSM6DSV80X_GY_Mode::HIGH_PERFORMANCE;

    config.enable_hg_xl = true;
    config.hg_range     = LSM6DSV80X_HG_Range::RANGE_80G;
    config.hg_odr       = LSM6DSV80X_HG_ODR::HG_ODR_480HZ;
    config.hg_mode      = LSM6DSV80X_HG_Mode::HIGH_PERFORMANCE;

    config.xl_range = LSM6DSV80X_XL_Range::RANGE_4G;
    config.xl_odr   = LSM6DSV80X_ODR::ODR_120HZ;
    config.xl_mode  = LSM6DSV80X_XL_Mode::HIGH_PERFORMANCE;

    config.enable_bdu        = true;
    config.enable_timestamp  = false;
    config.enable_drdy_int1  = false;
    config.enable_fifo       = false;
    config.fifo_mode         = LSM6DSV80X_FIFO_Mode::BYPASS;
    config.fifo_watermark    = 0;

    return (_imu.init(config) == LSM6DSV80X_Status::OK);
}

// ─── odrForHz ─────────────────────────────────────────────────────────────────

LSM6DSV80X_ODR LSM6DSV80X_Acquisition::odrForHz(uint8_t hz)
{
    if (hz <= 1)   return LSM6DSV80X_ODR::ODR_1HZ875;
    if (hz <= 7)   return LSM6DSV80X_ODR::ODR_7HZ5;
    if (hz <= 15)  return LSM6DSV80X_ODR::ODR_15HZ;
    if (hz <= 30)  return LSM6DSV80X_ODR::ODR_30HZ;
    if (hz <= 60)  return LSM6DSV80X_ODR::ODR_60HZ;
    if (hz <= 120) return LSM6DSV80X_ODR::ODR_120HZ;
    return             LSM6DSV80X_ODR::ODR_240HZ;
}