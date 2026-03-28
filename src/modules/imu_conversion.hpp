#ifndef IMU_CONVERSION_HPP
#define IMU_CONVERSION_HPP

#include <cstdint>
#include "drivers/kx134.hpp"

/* ============================================================
   IMU_Data
   Physical acceleration data in g.
   Independent from raw sensor ADC counts.
   ============================================================ */

struct IMU_Data
{
    float x_g;
    float y_g;
    float z_g;
};

/* ============================================================
   IMUConversion
   Converts raw KX134 sensor data (int16_t counts)
   into physical acceleration values in g.

   Responsibilities:
    - Apply range-based scaling
    - Apply calibration offsets
    - Perform deterministic math conversion

   ============================================================ */

class IMUConversion
{
public:

    /**
     * @brief Construct conversion object with initial range.
     * @param range Configured accelerometer full-scale range.
     */
    explicit IMUConversion(KX134_Range range) noexcept;

    /**
     * @brief Update sensor range (recomputes scale factor).
     */
    void setRange(KX134_Range range) noexcept;

    /**
     * @brief Get currently configured range.
     */
    [[nodiscard]] KX134_Range getRange() const noexcept;

    /**
     * @brief Set raw offset calibration values.
     *        Offsets are subtracted before scaling.
     */
    void setOffsets(int16_t x_off,
                    int16_t y_off,
                    int16_t z_off) noexcept;

    /**
     * @brief Clear all calibration offsets.
     */
    void clearOffsets() noexcept;

    /**
     * @brief Convert raw IMU data to acceleration (g).
     * @param raw Raw sensor ADC counts.
     * @return Converted physical acceleration.
     */
    [[nodiscard]] IMU_Data convert(const KX134_Raw& raw) const noexcept;

    /**
     * @brief Get current scale factor (g per LSB).
     */
    [[nodiscard]] float getScale() const noexcept;

private:

    KX134_Range _range;
    float       _scale;

    int16_t _x_offset;
    int16_t _y_offset;
    int16_t _z_offset;

    /**
     * @brief Recalculate scale factor from range.
     *        KX134 resolution: 16-bit signed (±32768 counts).
     */
    void updateScale() noexcept;

    /**
     * @brief Map KX134_Range enum to full-scale float value.
     *        Single source of truth for the range → g mapping.
     */
    static float rangeToFullScale(KX134_Range range) noexcept;
};


/* ============================================================
   MovingAverage
   Low-pass filter using a fixed-size ring buffer.

   Responsibilities:
    - Smooth x/y/z acceleration data over N samples
    - No heap allocation — fixed array on stack/BSS
    - Running sum avoids re-summing on every update

   Non-responsibilities:
    - No I2C access
    - No timing
    - No unit conversion

   Window size: WINDOW (compile-time constant, default 50)
   At 50Hz sampling, WINDOW=50 averages the last 1 second.
   ============================================================ */

class MovingAverage
{
public:

    static constexpr uint8_t WINDOW = 50;

    /**
     * @brief Construct filter with all buffers zeroed.
     */
    MovingAverage() noexcept;

    /**
     * @brief Push a new sample into the filter.
     * @param data Converted acceleration in g.
     * @return Filtered acceleration — average of last WINDOW samples.
     */
    IMU_Data update(const IMU_Data& data) noexcept;

    /**
     * @brief Reset all buffers and sums to zero.
     *        Call after calibration to flush stale samples.
     */
    void reset() noexcept;

    /**
     * @brief True once WINDOW samples have been collected.
     *        Before this point, output is the average of fewer samples.
     */
    [[nodiscard]] bool isWarmedUp() const noexcept;

private:

    float   _buf_x[WINDOW];
    float   _buf_y[WINDOW];
    float   _buf_z[WINDOW];

    float   _sum_x;     // running sum — avoids O(N) re-sum each update
    float   _sum_y;
    float   _sum_z;

    uint8_t _head;      // index of next write position (ring buffer)
    uint8_t _count;     // number of valid samples collected so far (0 → WINDOW)
};

#endif