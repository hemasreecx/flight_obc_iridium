#include "ls_conversion.hpp"

/* ============================================================
   LSM6DSV80X_Conversion
   ============================================================ */

// ─── Constructor ──────────────────────────────────────────────────────────────

LSM6DSV80X_Conversion::LSM6DSV80X_Conversion(LSM6DSV80X_GY_Range gy_range,
                                             LSM6DSV80X_HG_Range hg_range) noexcept
    : _gy_range(gy_range)
    , _hg_range(hg_range)
    , _gy_scale(gyFullScaleG(gy_range)  / 32768.0f)
    , _hg_scale(hgFullScaleG(hg_range)  / 32768.0f)
    , _gy_x_off(0), _gy_y_off(0), _gy_z_off(0)
    , _hg_x_off(0), _hg_y_off(0), _hg_z_off(0)
{
}

// ─── Range setters ────────────────────────────────────────────────────────────

void LSM6DSV80X_Conversion::setGYRange(LSM6DSV80X_GY_Range range) noexcept
{
    _gy_range = range;
    updateGYScale();
}

void LSM6DSV80X_Conversion::setHGRange(LSM6DSV80X_HG_Range range) noexcept
{
    _hg_range = range;
    updateHGScale();
}

LSM6DSV80X_GY_Range LSM6DSV80X_Conversion::getGYRange() const noexcept { return _gy_range; }
LSM6DSV80X_HG_Range LSM6DSV80X_Conversion::getHGRange() const noexcept { return _hg_range; }

// ─── Offset setters ───────────────────────────────────────────────────────────

void LSM6DSV80X_Conversion::setGYOffsets(int16_t x, int16_t y, int16_t z) noexcept
{
    _gy_x_off = x;  _gy_y_off = y;  _gy_z_off = z;
}

void LSM6DSV80X_Conversion::setHGOffsets(int16_t x, int16_t y, int16_t z) noexcept
{
    _hg_x_off = x;  _hg_y_off = y;  _hg_z_off = z;
}

void LSM6DSV80X_Conversion::clearGYOffsets() noexcept { _gy_x_off = _gy_y_off = _gy_z_off = 0; }
void LSM6DSV80X_Conversion::clearHGOffsets() noexcept { _hg_x_off = _hg_y_off = _hg_z_off = 0; }

// ─── Conversion ───────────────────────────────────────────────────────────────
//
// Pattern for both channels:
//   1. Promote raw int16 to int32 before subtracting offset.
//      Prevents silent overflow when raw is near ±32768.
//   2. Multiply by pre-computed scale to get physical units.
//
// Output units:
//   convertGY → dps  (degrees per second)
//   convertHG → g    (standard gravity)

LSM6DSV80X_Data LSM6DSV80X_Conversion::convertGY(const LSM6DSV80X_RawGY& raw) const noexcept
{
    int32_t cx = static_cast<int32_t>(raw.x) - _gy_x_off;
    int32_t cy = static_cast<int32_t>(raw.y) - _gy_y_off;
    int32_t cz = static_cast<int32_t>(raw.z) - _gy_z_off;

    return {
        static_cast<float>(cx) * _gy_scale,
        static_cast<float>(cy) * _gy_scale,
        static_cast<float>(cz) * _gy_scale
    };
}

LSM6DSV80X_Data LSM6DSV80X_Conversion::convertHG(const LSM6DSV80X_RawHG& raw) const noexcept
{
    int32_t cx = static_cast<int32_t>(raw.x) - _hg_x_off;
    int32_t cy = static_cast<int32_t>(raw.y) - _hg_y_off;
    int32_t cz = static_cast<int32_t>(raw.z) - _hg_z_off;

    return {
        static_cast<float>(cx) * _hg_scale,
        static_cast<float>(cy) * _hg_scale,
        static_cast<float>(cz) * _hg_scale
    };
}

// ─── Scale inspection ─────────────────────────────────────────────────────────

float LSM6DSV80X_Conversion::getGYScale() const noexcept { return _gy_scale; }
float LSM6DSV80X_Conversion::getHGScale() const noexcept { return _hg_scale; }

// ─── Private: full-scale lookup tables ───────────────────────────────────────
//
// Sensitivity = full_scale / 32768.0f  (16-bit signed ADC, ±32768 counts).
//
// Output is in g (not mg) and dps (not mdps).
// Derivation: FS_mg / 1000 → g,  FS_mdps / 1000 → dps
//
// GY  (dps):
//   125dps  → 125.0f
//   250dps  → 250.0f
//   500dps  → 500.0f
//   1000dps → 1000.0f
//   2000dps → 2000.0f
//   4000dps → 4000.0f
//
// HG  (g):
//   32g  → 32.0f
//   64g  → 64.0f
//   80g  → 80.0f

float LSM6DSV80X_Conversion::gyFullScaleG(LSM6DSV80X_GY_Range range) noexcept
{
    switch (range)
    {
        case LSM6DSV80X_GY_Range::RANGE_125DPS:  return   125.0f;
        case LSM6DSV80X_GY_Range::RANGE_250DPS:  return   250.0f;
        case LSM6DSV80X_GY_Range::RANGE_500DPS:  return   500.0f;
        case LSM6DSV80X_GY_Range::RANGE_1000DPS: return  1000.0f;
        case LSM6DSV80X_GY_Range::RANGE_2000DPS: return  2000.0f;
        case LSM6DSV80X_GY_Range::RANGE_4000DPS: return  4000.0f;
        default:                                  return  1000.0f;  // safe fallback
    }
}

float LSM6DSV80X_Conversion::hgFullScaleG(LSM6DSV80X_HG_Range range) noexcept
{
    switch (range)
    {
        case LSM6DSV80X_HG_Range::RANGE_32G: return  32.0f;
        case LSM6DSV80X_HG_Range::RANGE_64G: return  64.0f;
        case LSM6DSV80X_HG_Range::RANGE_80G: return  80.0f;
        default:                              return  80.0f;  // safe fallback
    }
}

// ─── Private: scale updaters ──────────────────────────────────────────────────

void LSM6DSV80X_Conversion::updateGYScale() noexcept
{
    _gy_scale = gyFullScaleG(_gy_range) / 32768.0f;
}

void LSM6DSV80X_Conversion::updateHGScale() noexcept
{
    _hg_scale = hgFullScaleG(_hg_range) / 32768.0f;
}


/* ============================================================
   LSM6DSV80X_Filter
   ============================================================
   Ring-buffer moving-average low-pass filter.

   Ring buffer mechanics:
     _head  — slot where the NEXT sample will be written
     _count — valid entries so far (0 → WINDOW, stays at WINDOW)

   Running-sum trick:
     Keep _sum_x/y/z updated incrementally per update:
       sum += new_value - old_value_being_replaced
     This keeps every call O(1) regardless of WINDOW size.

   Warmup:
     For the first WINDOW samples, _count < WINDOW so the
     average is computed over fewer points. isWarmedUp()
     tells the caller when all WINDOW slots are populated.
   ============================================================ */

LSM6DSV80X_Filter::LSM6DSV80X_Filter() noexcept
    : _sum_x(0.0f)
    , _sum_y(0.0f)
    , _sum_z(0.0f)
    , _head(0)
    , _count(0)
{
    // IEEE 754: all-zero bits == 0.0f — memset is safe on float arrays
    memset(_buf_x, 0, sizeof(_buf_x));
    memset(_buf_y, 0, sizeof(_buf_y));
    memset(_buf_z, 0, sizeof(_buf_z));
}

LSM6DSV80X_Data LSM6DSV80X_Filter::update(const LSM6DSV80X_Data& data) noexcept
{
    // Step 1: remove the oldest value from the running sum
    _sum_x -= _buf_x[_head];
    _sum_y -= _buf_y[_head];
    _sum_z -= _buf_z[_head];

    // Step 2: overwrite the oldest slot with the new sample
    _buf_x[_head] = data.x;
    _buf_y[_head] = data.y;
    _buf_z[_head] = data.z;

    // Step 3: add new sample to running sum
    _sum_x += data.x;
    _sum_y += data.y;
    _sum_z += data.z;

    // Step 4: advance head — wraps at WINDOW
    _head = static_cast<uint8_t>((_head + 1) % WINDOW);

    // Step 5: count up to WINDOW (tracks warmup)
    if (_count < WINDOW)
        _count++;

    // Step 6: divide by valid sample count
    float inv = 1.0f / static_cast<float>(_count);

    return {
        _sum_x * inv,
        _sum_y * inv,
        _sum_z * inv
    };
}

void LSM6DSV80X_Filter::reset() noexcept
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

bool LSM6DSV80X_Filter::isWarmedUp() const noexcept
{
    return (_count == WINDOW);
}