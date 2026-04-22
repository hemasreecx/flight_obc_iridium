#include "imu_acquisition.hpp"
#include "config.hpp"
#include "pico/stdlib.h"
#include "imu_conversion.hpp"
#include "kx_data_logger.hpp"

IMUAcquisition::IMUAcquisition(KX134& imu, DataLogger& logger)
    : _imu(imu),
      _rate_hz(50),
      _last_sample_time_ms(0),
      _converter(KX134_Range::RANGE_64G),
      _filter(),                          // zero-initialised by MovingAverage()
      _logger(logger),
      _latest{},
      _has_sample(false)
{
}

bool IMUAcquisition::init()
{
    KX134_Config config{};
    config.range                       = KX134_Range::RANGE_64G;
    config.odr                         = KX134_ODR::KX134_ODR_50HZ;
    config.performance_mode            = KX134_Performance::HIGH_PERFORMANCE;
    config.enable_data_ready_interrupt = false;
    config.enable_fifo                 = false;

    if (_imu.init(config) != KX134_Status::OK)
        return false;

    _rate_hz             = 50;
    _last_sample_time_ms = to_ms_since_boot(get_absolute_time());
    _latest              = {};
    _has_sample          = false;

    return true;
}

/* ============================================================
   task()
   Called every 1ms from main_loop().
   Internal timer fires at the configured sample rate (50Hz).

   Data pipeline per sample:
     readRaw() → convert() → filter.update() → logger.log()

   The filter sits between conversion and logging.
   Raw and converted values are both available to the logger
   so it can choose which to print based on LoggerMode.
   ============================================================ */

KX134_Status IMUAcquisition::task()
{
    if (_rate_hz == 0)
        return KX134_Status::ERR_CONFIG;

    uint64_t now       = to_ms_since_boot(get_absolute_time());
    uint64_t period_ms = 1000ULL / _rate_hz;

    if ((now - _last_sample_time_ms) < period_ms)
        return KX134_Status::OK;   // not time yet — not an error

    _last_sample_time_ms = now;

    // Step 1: read raw ADC counts from sensor
    KX134_Raw    raw;
    KX134_Status status = _imu.readRaw(raw);

    if (status != KX134_Status::OK)
        return status;

    // Step 2: apply calibration offsets and scale to g
    IMU_Data converted = _converter.convert(raw);

    // Step 3: push through moving average filter
    //         filtered holds the smoothed output
    IMU_Data filtered = _filter.update(converted);

    _latest.raw_x = raw.x;
    _latest.raw_y = raw.y;
    _latest.raw_z = raw.z;
    _latest.x_g = filtered.x_g;
    _latest.y_g = filtered.y_g;
    _latest.z_g = filtered.z_g;
    _latest.timestamp_ms = now;
    _has_sample = true;

    // Step 4: log — RAW mode logs raw counts,
    //               CONVERTED mode logs filtered g values
    // FIX: timestamp_us removed — DataLogger::log() no longer
    //      accepts it. Was unused anyway.
#if SENSOR_SERIAL_STREAM_ENABLE
    _logger.log(raw, filtered);
#endif

    return KX134_Status::OK;
}

void IMUAcquisition::setRateHz(uint32_t hz)
{
    if (hz == 0)  hz = 1;
    if (hz > 400) hz = 400;

    _rate_hz = hz;
    configureODR(hz);
}

void IMUAcquisition::setCalibrationOffsets(int16_t x, int16_t y, int16_t z)
{
    _converter.setOffsets(x, y, z);
}

KX134_Range IMUAcquisition::getConverterRange() const
{
    return _converter.getRange();
}

float IMUAcquisition::getConverterScale() const
{
    return _converter.getScale();
}

const IMUSample& IMUAcquisition::getLatestSample() const
{
    return _latest;
}

bool IMUAcquisition::hasSample() const
{
    return _has_sample;
}

/* ============================================================
   resetFilter()
   Flushes the moving average ring buffer.

   Must be called after setCalibrationOffsets() so that samples
   measured under the old offsets don't pollute the filtered
   output. Call order in main.cpp:

     setCalibrationOffsets(x, y, z)
     resetFilter()                   ← flush stale samples
     (start logging)
   ============================================================ */

void IMUAcquisition::resetFilter()
{
    _filter.reset();
}

/* ============================================================
   configureODR()
   NOTE: calls _imu.init() which does a full chip reset.
   Known issue — acceptable because setRateHz() is only called
   once during system_init() before sampling begins.
   ============================================================ */

bool IMUAcquisition::configureODR(uint32_t hz)
{
    KX134_ODR odr = KX134_ODR::KX134_ODR_50HZ;

    switch (hz)
    {
        case 12:  odr = KX134_ODR::KX134_ODR_12_5HZ; break;
        case 25:  odr = KX134_ODR::KX134_ODR_25HZ;   break;
        case 50:  odr = KX134_ODR::KX134_ODR_50HZ;   break;
        case 100: odr = KX134_ODR::KX134_ODR_100HZ;  break;
        case 200: odr = KX134_ODR::KX134_ODR_200HZ;  break;
        case 400: odr = KX134_ODR::KX134_ODR_400HZ;  break;
        default:  odr = KX134_ODR::KX134_ODR_50HZ;   break;
    }

    KX134_Config config{};
    config.range                       = KX134_Range::RANGE_64G;
    config.odr                         = odr;
    config.performance_mode            = KX134_Performance::HIGH_PERFORMANCE;
    config.enable_data_ready_interrupt = false;
    config.enable_fifo                 = false;

    return (_imu.init(config) == KX134_Status::OK);
}