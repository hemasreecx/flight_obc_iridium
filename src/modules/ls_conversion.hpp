#pragma once

#include <stdint.h>
#include <string.h>
#include "LSM6DSV80X.hpp"

// ─── Output Data Struct ───────────────────────────────────────────────────────
// GY  → x / y / z in dps  (degrees per second)
// HG  → x / y / z in g    (standard gravity)
// Both channels share the same struct — unit depends on which
// converter method produced the value.

struct LSM6DSV80X_Data
{
    float x;
    float y;
    float z;
};

// ─── LSM6DSV80X_Conversion ───────────────────────────────────────────────────
//
// Converts raw int16 ADC counts → physical units for gyro and high-G channels.
// Low-G accelerometer is not used and has been removed entirely.
//
// Scale derivation:
//   GY:  sensitivity = full_scale_dps  / 32768.0f   → result in dps
//   HG:  sensitivity = full_scale_g    / 32768.0f   → result in g
//
// Offsets are in raw ADC counts, subtracted before scaling.
// Promoted to int32 before subtraction to prevent int16 overflow.

class LSM6DSV80X_Conversion
{
public:
    LSM6DSV80X_Conversion(LSM6DSV80X_GY_Range gy_range,
                          LSM6DSV80X_HG_Range hg_range) noexcept;

    // ── Range updates ─────────────────────────────────────────────────────────
    void setGYRange(LSM6DSV80X_GY_Range range) noexcept;
    void setHGRange(LSM6DSV80X_HG_Range range) noexcept;

    LSM6DSV80X_GY_Range getGYRange() const noexcept;
    LSM6DSV80X_HG_Range getHGRange() const noexcept;

    // ── Offsets (raw ADC counts) ───────────────────────────────────────────────
    void setGYOffsets(int16_t x, int16_t y, int16_t z) noexcept;
    void setHGOffsets(int16_t x, int16_t y, int16_t z) noexcept;

    void clearGYOffsets() noexcept;
    void clearHGOffsets() noexcept;

    // ── Conversion ────────────────────────────────────────────────────────────
    // convertGY → dps  (degrees per second)
    // convertHG → g    (standard gravity)
    LSM6DSV80X_Data convertGY(const LSM6DSV80X_RawGY& raw) const noexcept;
    LSM6DSV80X_Data convertHG(const LSM6DSV80X_RawHG& raw) const noexcept;

    // ── Scale inspection ──────────────────────────────────────────────────────
    float getGYScale() const noexcept;   // dps / count
    float getHGScale() const noexcept;   // g   / count

private:
    LSM6DSV80X_GY_Range _gy_range;
    LSM6DSV80X_HG_Range _hg_range;

    float _gy_scale;   // dps / count
    float _hg_scale;   // g   / count

    int16_t _gy_x_off, _gy_y_off, _gy_z_off;
    int16_t _hg_x_off, _hg_y_off, _hg_z_off;

    // Single source of truth for range → full-scale value
    static float gyFullScaleG (LSM6DSV80X_GY_Range range) noexcept;
    static float hgFullScaleG (LSM6DSV80X_HG_Range range) noexcept;

    void updateGYScale() noexcept;
    void updateHGScale() noexcept;
};

// ─── LSM6DSV80X_Filter ────────────────────────────────────────────────────────
//
// Ring-buffer moving-average low-pass filter for LSM6DSV80X_Data.
//
// O(1) per update via running-sum trick:
//   sum += new_value - value_being_replaced
//
// WINDOW = 8 at 60 Hz:
//   effective cut-off ≈ ODR / (2 * WINDOW) = 60 / 16 ≈ 3.75 Hz

class LSM6DSV80X_Filter
{
public:
    static constexpr uint8_t WINDOW = 8;

    LSM6DSV80X_Filter() noexcept;

    LSM6DSV80X_Data update(const LSM6DSV80X_Data& data) noexcept;
    void            reset()      noexcept;
    bool            isWarmedUp() const noexcept;

private:
    float   _buf_x[WINDOW];
    float   _buf_y[WINDOW];
    float   _buf_z[WINDOW];

    float   _sum_x;
    float   _sum_y;
    float   _sum_z;

    uint8_t _head;
    uint8_t _count;
};