// mag_conversion.hpp
#ifndef MAG_CONVERSION_HPP
#define MAG_CONVERSION_HPP

#include <stdint.h>

/**
 * @brief Handles all unit conversion for the QMC5883L magnetometer.
 *
 *        Two responsibilities:
 *          1. Raw counts → Gauss   (toGauss)
 *          2. Filtered XY Gauss   → compass heading 0–360° (toHeading)
 *
 *        Fixed to ±8 Gauss range:
 *          QMC5883L datasheet — ±8G range gives 3000 LSB/Gauss
 *          i.e. 1 count = 1/3000 Gauss ≈ 0.000333 G/LSB
 */
class MagConversion
{
public:
    MagConversion() = default;

    /**
     * @brief Convert a single raw axis count to Gauss.
     * @param counts  Raw int16 reading from sensor (post-calibration)
     * @return        Field strength in Gauss
     *
     * Example:
     *   counts = 3000  →  1.000 G
     *   counts = -1500 → -0.500 G
     *   counts = 0     →  0.000 G
     */
    float toGauss(float counts) const;

    /**
     * @brief Compute 2D compass heading from filtered X and Y Gauss values.
     *        Assumes sensor is mounted level (XY plane = horizontal).
     *        Returns degrees in [0, 360).
     *
     * @param x_gauss  Filtered X axis in Gauss
     * @param y_gauss  Filtered Y axis in Gauss
     * @return         Heading in degrees, 0 = magnetic north, clockwise +ve
     *
     * Example:
     *   x=1.0, y=0.0  →   0.0°  (pointing north)
     *   x=0.0, y=1.0  →  90.0°  (pointing east)
     *   x=-1.0, y=0.0 → 180.0°  (pointing south)
     *   x=0.0, y=-1.0 → 270.0°  (pointing west)
     */
    float toHeading(float x_gauss, float y_gauss) const;

    // ── Accessors — useful for logging/debugging ──────────────────────────────

    /**
     * @brief Returns the scale factor being used (LSB per Gauss).
     *        Fixed at 3000.0f for ±8G range.
     */
    float getScaleLSBperGauss() const;

private:
    // QMC5883L datasheet, Table 7:
    //   RNG = 8G  →  sensitivity = 3000 LSB/Gauss
    static constexpr float SCALE_LSB_PER_GAUSS = 3000.0f;
};

#endif