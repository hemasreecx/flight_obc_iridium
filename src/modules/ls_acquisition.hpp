#pragma once

#include <stdint.h>
#include "LSM6DSV80X.hpp"
#include "ls_conversion.hpp"   // LSM6DSV80X_Data, LSM6DSV80X_Conversion,
                                        // LSM6DSV80X_Filter — owns all shared types

// ─── Acquisition class ────────────────────────────────────────────────────────
//
// Ties together the IMU driver, converter, and filters into a single
// rate-limited polling interface.
//
// Temperature:
//   Raw int16 from the sensor. Converted via: (raw / 256.0f) + 25.0f = °C
//   The temp offset (set via setTempOffset) is subtracted from raw before
//   the value is stored, correcting the sensor's baseline error.
//   Temperature is NOT filtered — it changes slowly enough that raw
//   offset-corrected counts are sufficient.

class LSM6DSV80X_Acquisition
{
public:
    LSM6DSV80X_Acquisition(LSM6DSV80X& imu, uint8_t rate_hz = 60);

    // Call once at startup
    bool init();

    // Call regularly from the main loop; returns OK if nothing went wrong
    LSM6DSV80X_Status task();

    // Runtime rate change — takes effect immediately
    void setRateHz(uint8_t hz);

    // Calibration offset passthrough → forwarded to LSM6DSV80X_Conversion
    void setGYOffsets(int16_t x, int16_t y, int16_t z);
    void setHGOffsets(int16_t x, int16_t y, int16_t z);
    void clearGYOffsets();
    void clearHGOffsets();

    // Temperature offset — subtracted from raw before storing
    // offset = average raw at calibration time (zero-point correction)
    void    setTempOffset(int16_t offset);
    void    clearTempOffset();
    int16_t getTempOffset() const { return _temp_offset; }

    // Latest filtered samples (updated each time task() fires)
    LSM6DSV80X_Data getFilteredGY()  const { return _filtered_gy; }
    LSM6DSV80X_Data getFilteredHG()  const { return _filtered_hg; }

    // Latest raw counts
    LSM6DSV80X_RawGY getRawGY()  const { return _raw_gy; }
    LSM6DSV80X_RawHG getRawHG()  const { return _raw_hg; }

    // Temperature: offset-corrected raw count
    // Convert to °C: (getRawTemp() / 256.0f) + 25.0f
    int16_t getRawTemp() const { return _raw_temp; }

    void resetFilters();

private:
    LSM6DSV80X&           _imu;
    uint8_t               _rate_hz;
    uint32_t              _last_sample_time_us;

    LSM6DSV80X_Conversion _converter;
    LSM6DSV80X_Filter     _gy_filter;
    LSM6DSV80X_Filter     _hg_filter;

    LSM6DSV80X_RawGY      _raw_gy;
    LSM6DSV80X_RawHG      _raw_hg;
    int16_t               _raw_temp;      // offset-corrected temperature count
    int16_t               _temp_offset;   // subtracted from sensor raw before storing

    LSM6DSV80X_Data       _filtered_gy;
    LSM6DSV80X_Data       _filtered_hg;

    bool reconfigureODR(uint8_t hz);
    static LSM6DSV80X_ODR odrForHz(uint8_t hz);
};