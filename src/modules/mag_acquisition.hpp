#ifndef MAG_ACQUISITION_HPP
#define MAG_ACQUISITION_HPP

#include <stdint.h>
#include "qmc5883l.hpp"
#include "mag_conversion.hpp"
#include "qmc_data_logger.hpp"   // MovingAverage + MagSample + MagDataLogger + MagCalibData

// ── Calibration structs ───────────────────────────────────────────────────────

struct MagHardIron
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

struct MagSoftIron
{
    float m[9] = { 1,0,0,
                   0,1,0,
                   0,0,1 };
};

// ── Main class ────────────────────────────────────────────────────────────────

class MagAcquisition
{
public:
    MagAcquisition(QMC5883L& sensor, MagDataLogger& logger);

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    bool            init(const QMC5883L_Config& config);
    QMC5883L_Status task();

    // ── Calibration setters ───────────────────────────────────────────────────

    /**
     * @brief Set hard-iron offset correction (per-axis additive bias).
     *        Resets filter to flush stale pre-calibration samples.
     */
    void setHardIronOffsets(int16_t x, int16_t y, int16_t z);

    /**
     * @brief Set soft-iron correction matrix (3x3, row-major).
     *        Resets filter for same reason as above.
     *        Pass identity matrix to disable soft-iron correction.
     */
    void setSoftIronMatrix(const float m[9]);

    /**
     * @brief Set temperature offset correction in degrees Celsius.
     *        Added to raw sensor temperature before storing in MagSample.
     *        Use when sensor runs warm due to nearby components.
     *        Default is 0.0f (no correction).
     *
     * Example:
     *   sensor reads 32.5C but true temp is 30.0C
     *   → setTempOffset(-2.5f)
     */
    void setTempOffset(float offset_c);

    // ── Filter ────────────────────────────────────────────────────────────────

    /**
     * @brief Flush all stale samples from moving-average ring buffers.
     *        Call after sensor power-cycle or manual calibration change.
     */
    void resetFilter();

    // ── Getters ───────────────────────────────────────────────────────────────

    /**
     * @brief Returns most recent fully-processed sample.
     *        Valid only after at least one successful task() call.
     */
    const MagSample& getLatestSample() const;

    float    getHeadingDeg()    const;
    float    getTemperature()   const;
    uint32_t getSampleCount()   const;
    uint32_t getSkippedCount()  const;
    uint32_t getOverflowCount() const;

    void resetCounters();

private:
    bool readAndProcess();
    void applyCalibration(int16_t rx, int16_t ry, int16_t rz,
                          float& cx, float& cy, float& cz) const;

    QMC5883L&      _sensor;
    MagDataLogger& _logger;

    MagHardIron   _hard_iron;
    MagSoftIron   _soft_iron;

    float         _temp_offset;   // temperature correction in °C, default 0.0f

    MagConversion _converter;

    MagMovingAverage _filter_x;
    MagMovingAverage _filter_y;
    MagMovingAverage _filter_z;

    MagSample _latest;

    uint32_t _sample_count;
    uint32_t _skipped_count;
    uint32_t _overflow_count;
};

#endif