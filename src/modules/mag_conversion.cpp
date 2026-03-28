// mag_conversion.cpp
#include "mag_conversion.hpp"
#include <cmath>    // atan2f, M_PI

// ── toGauss() ─────────────────────────────────────────────────────────────────

float MagConversion::toGauss(float counts) const
{
    // ±8G range: sensor outputs 3000 counts per 1 Gauss
    //
    // counts → Gauss:  value = counts / 3000
    //
    // Concrete examples at ±8G:
    //   Max positive:  32767 /  3000 =  +10.92 G  (hard saturation)
    //   Typical earth:  ~500 /  3000 =  +0.167 G  (earth's field ~0.25–0.65 G)
    //   Zero field:        0 /  3000 =   0.000 G
    //   Typical earth:  ~500 /  3000 =  -0.167 G
    //   Max negative: -32768 /  3000 =  -10.92 G  (hard saturation)
    return counts / SCALE_LSB_PER_GAUSS;
}

// ── toHeading() ───────────────────────────────────────────────────────────────

float MagConversion::toHeading(float x_gauss, float y_gauss) const
{
    // ── Step 1: atan2 gives angle from +X axis, in radians [-π, +π] ──────────
    //
    // Why atan2(y, x) and not atan(y/x)?
    //   atan(y/x) loses quadrant information — it can't tell NE from SW.
    //   atan2 uses the signs of both arguments to place the angle
    //   in the correct quadrant across all 360°.
    //
    //      +X
    //       │  atan2(0, 1)  =  0 rad  =   0°
    //  ─────┼─────  +Y
    //       │  atan2(1, 0)  = +π/2   =  90°
    //      -X
    //         atan2(0, -1) = ±π     = 180°
    //         atan2(-1, 0) = -π/2   = 270° (after correction below)
    //
    float heading_rad = atan2f(y_gauss, x_gauss);

    // ── Step 2: Convert radians → degrees ────────────────────────────────────
    float heading_deg = heading_rad * (180.0f / static_cast<float>(M_PI));

    // ── Step 3: Shift [-180, +180] → [0, 360] ────────────────────────────────
    // atan2 returns negative angles for the southern half-plane.
    // A compass heading is always positive, so we add 360° to any
    // negative result.
    //
    // Example:
    //   atan2(-1, 0) = -90°  →  -90 + 360 = 270°  ✓ (west)
    //   atan2(1, 0)  = +90°  →  stays 90°          ✓ (east)
    if (heading_deg < 0.0f)
        heading_deg += 360.0f;

    return heading_deg;
}

// ── getScaleLSBperGauss() ─────────────────────────────────────────────────────

float MagConversion::getScaleLSBperGauss() const
{
    return SCALE_LSB_PER_GAUSS;
}