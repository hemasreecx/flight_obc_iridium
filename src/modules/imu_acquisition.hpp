#ifndef IMU_ACQUISITION_HPP
#define IMU_ACQUISITION_HPP

#include <stdint.h>
#include "kx134.hpp"
#include "imu_conversion.hpp"
#include "kx_data_logger.hpp"

class IMUAcquisition
{
public:
    IMUAcquisition(KX134& imu, DataLogger& logger);

    bool         init();
    KX134_Status task();
    void         setRateHz(uint32_t hz);
    void         setCalibrationOffsets(int16_t x, int16_t y, int16_t z);
    KX134_Range  getConverterRange() const;
    float        getConverterScale() const;

    /**
     * @brief Reset the moving average filter.
     *        Call after new calibration offsets are applied so
     *        stale pre-calibration samples are flushed from
     *        the ring buffer before logging starts.
     */
    void resetFilter();

private:
    KX134&        _imu;
    uint32_t      _rate_hz;
    uint64_t      _last_sample_time_ms;
    IMUConversion _converter;
    MovingAverage _filter;        // owned — low-pass filter, lives here
    DataLogger&   _logger;

    bool configureODR(uint32_t hz);
};

#endif