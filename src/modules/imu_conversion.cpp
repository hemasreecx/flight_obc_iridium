#include "imu_conversion.hpp"
#include <string.h>   // memset

/* ============================================================
   IMUConversion
   ============================================================ */

IMUConversion::IMUConversion(KX134_Range range) noexcept
    : _range(range),
      _scale(rangeToFullScale(range) / 32768.0f),  // FIX: computed directly,
      _x_offset(0),                                 //      not set to 0.0f then
      _y_offset(0),                                 //      overwritten
      _z_offset(0)
{
}

void IMUConversion::setRange(KX134_Range range) noexcept
{
    _range = range;
    updateScale();
}

KX134_Range IMUConversion::getRange() const noexcept
{
    return _range;
}

void IMUConversion::setOffsets(int16_t x_off,
                               int16_t y_off,
                               int16_t z_off) noexcept
{
    _x_offset = x_off;
    _y_offset = y_off;
    _z_offset = z_off;
}

void IMUConversion::clearOffsets() noexcept
{
    _x_offset = 0;
    _y_offset = 0;
    _z_offset = 0;
}

IMU_Data IMUConversion::convert(const KX134_Raw& raw) const noexcept
{
    IMU_Data data;

    // Promote to 32-bit before subtracting to avoid int16 overflow.
    // Example: raw.x = -32768, _x_offset = +100
    //          result = -32868 — fits int32, overflows int16.
    int32_t x_corr = static_cast<int32_t>(raw.x) - _x_offset;
    int32_t y_corr = static_cast<int32_t>(raw.y) - _y_offset;
    int32_t z_corr = static_cast<int32_t>(raw.z) - _z_offset;

    data.x_g = static_cast<float>(x_corr) * _scale;
    data.y_g = static_cast<float>(y_corr) * _scale;
    data.z_g = static_cast<float>(z_corr) * _scale;

    return data;
}

float IMUConversion::getScale() const noexcept
{
    return _scale;
}

/* ============================================================
   PRIVATE
   ============================================================ */

// FIX: rangeToFullScale() is the single source of truth for
// the range → g mapping. Both the constructor and updateScale()
// use it so the logic is never duplicated.
float IMUConversion::rangeToFullScale(KX134_Range range) noexcept
{
    switch (range)
    {
        case KX134_Range::RANGE_8G:  return 8.0f;
        case KX134_Range::RANGE_16G: return 16.0f;
        case KX134_Range::RANGE_32G: return 32.0f;
        case KX134_Range::RANGE_64G: return 64.0f;
        default:                     return 64.0f;  // safe fallback
    }
}

void IMUConversion::updateScale() noexcept
{
    // 16-bit signed range: ±32768 counts
    _scale = rangeToFullScale(_range) / 32768.0f;
}


/* ============================================================
   MovingAverage
   Ring buffer low-pass filter — one buffer per axis.

   Ring buffer mechanics:
     _head  — index where the NEXT sample will be written
     _count — how many valid entries exist (0 → WINDOW)

   Running sum trick:
     Instead of summing all WINDOW values every update (slow),
     we keep _sum_x/y/z updated incrementally:
       sum += new_value - old_value_being_replaced
     This makes every update O(1) regardless of window size.
   ============================================================ */

MovingAverage::MovingAverage() noexcept
    : _sum_x(0.0f),
      _sum_y(0.0f),
      _sum_z(0.0f),
      _head(0),
      _count(0)
{
    // Zero all three buffers — memset is safe for float arrays
    // since IEEE 754 represents 0.0f as all-zero bits.
    memset(_buf_x, 0, sizeof(_buf_x));
    memset(_buf_y, 0, sizeof(_buf_y));
    memset(_buf_z, 0, sizeof(_buf_z));
}

IMU_Data MovingAverage::update(const IMU_Data& data) noexcept
{
    // Step 1: subtract the oldest value from the running sum
    //         (before we overwrite it with the new value)
    _sum_x -= _buf_x[_head];
    _sum_y -= _buf_y[_head];
    _sum_z -= _buf_z[_head];

    // Step 2: write new value into the slot the oldest occupied
    _buf_x[_head] = data.x_g;
    _buf_y[_head] = data.y_g;
    _buf_z[_head] = data.z_g;

    // Step 3: add new value to running sum
    _sum_x += data.x_g;
    _sum_y += data.y_g;
    _sum_z += data.z_g;

    // Step 4: advance head — wraps back to 0 after WINDOW-1
    _head = static_cast<uint8_t>((_head + 1) % WINDOW);

    // Step 5: track count until buffer is full
    if (_count < WINDOW)
        _count++;

    // Step 6: divide by actual number of valid samples
    //         (less than WINDOW during warmup, WINDOW after)
    float inv = 1.0f / static_cast<float>(_count);

    IMU_Data filtered;
    filtered.x_g = _sum_x * inv;
    filtered.y_g = _sum_y * inv;
    filtered.z_g = _sum_z * inv;

    return filtered;
}

void MovingAverage::reset() noexcept
{
    memset(_buf_x, 0, sizeof(_buf_x));
    memset(_buf_y, 0, sizeof(_buf_y));
    memset(_buf_z, 0, sizeof(_buf_z));

    _sum_x = 0.0f;
    _sum_y = 0.0f;
    _sum_z = 0.0f;
    _head  = 0;
    _count = 0;
}

bool MovingAverage::isWarmedUp() const noexcept
{
    return (_count == WINDOW);
}