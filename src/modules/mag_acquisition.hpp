#ifndef MAG_ACQUISITION_HPP
#define MAG_ACQUISITION_HPP

#include <stdint.h>
#include "qmc5883l.hpp"
#include "mag_conversion.hpp"
#include "qmc_data_logger.hpp"   // MagMovingAverage + MagSample + MagDataLogger

// ── Calibration structs ───────────────────────────────────────────────────────

/**
 * @brief Hard-iron offsets (additive bias on each axis).
 *        Caused by permanent magnets near the sensor (motors, speaker etc.)
 *        Correct by subtracting these from raw counts before anything else.
 */
struct MagHardIron
{
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

/**
 * @brief Soft-iron correction matrix (3x3, row-major).
 *        Caused by soft magnetic materials that distort the field shape.
 *        Applied after hard-iron: corrected = W * (raw - hard_iron)
 *        Default is the identity matrix (no correction).
 *
 *        Layout:
 *          | m[0]  m[1]  m[2] |   | x |
 *          | m[3]  m[4]  m[5] | × | y |
 *          | m[6]  m[7]  m[8] |   | z |
 */
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
    /**
     * @param sensor  the QMC5883L driver (already constructed)
     * @param logger  shared MagDataLogger instance
     */
    MagAcquisition(QMC5883L& sensor, MagDataLogger& logger);

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /**
     * @brief Initialise sensor hardware and reset all state.
     *        Must be called before task().
     */
    bool init(const QMC5883L_Config& config);

    /**
     * @brief Call this repeatedly from your main loop.
     *        Behaviour depends on logger mode:
     *
     *        PRE_CALIB  → reads raw counts, logs via logRaw(), returns
     *        POST_CALIB → reads raw, applies hard-iron, logs via logCalibrated(), returns
     *        CONVERTED  → full pipeline: calibrate → Gauss → filter → heading → log
     *
     * @return QMC5883L_Status — lets caller detect errors each cycle.
     */
    QMC5883L_Status task();

    // ── Calibration setters ───────────────────────────────────────────────────

    /**
     * @brief Set hard-iron offset correction (per-axis additive bias).
     *        Automatically resets the moving-average filter so stale
     *        pre-calibration samples are flushed.
     */
    void setHardIronOffsets(int16_t x, int16_t y, int16_t z);

    /**
     * @brief Set soft-iron correction matrix (3×3, row-major).
     *        Also resets the filter for the same reason as above.
     *        Pass identity matrix to disable soft-iron correction.
     */
    void setSoftIronMatrix(const float m[9]);

    /**
     * @brief Set temperature offset for OBC thermal calibration.
     *        The QMC5883L temp register is not factory calibrated.
     *        Measure actual board temp with a thermal camera, then:
     *          offset = actual_temp - sensor_raw_reading
     *        Example: thermal camera = 30.5°C, sensor reads -2.26°C
     *          offset = 30.5 - (-2.26) = 32.76°C
     *        Default is 0.0f (no correction).
     */
    void setTempOffset(float offset_c);

    // ── Filter ────────────────────────────────────────────────────────────────

    /**
     * @brief Flush all stale samples from the moving-average ring buffer.
     *        Call explicitly if you change calibration without using the
     *        set*() helpers, or after a sensor power-cycle.
     */
    void resetFilter();

    // ── Getters ───────────────────────────────────────────────────────────────

    /**
     * @brief Returns the most recent fully-processed sample.
     *        Valid only after at least one successful task() call
     *        in CONVERTED mode.
     */
    const MagSample& getLatestSample() const;

    float    getHeadingDeg()    const;  // shortcut — latest heading
    float    getTemperature()   const;  // shortcut — latest calibrated temp
    uint32_t getSampleCount()   const;
    uint32_t getSkippedCount()  const;
    uint32_t getOverflowCount() const;

    void resetCounters();

private:
    // ── Core helpers ──────────────────────────────────────────────────────────

    /**
     * @brief Full pipeline — read raw, calibrate, Gauss, filter, heading.
     *        Only called in CONVERTED mode.
     *        Populates _latest.
     */
    bool readAndProcess(const QMC5883L_Raw& raw);

    /**
     * @brief Apply hard-iron offset + soft-iron matrix to raw counts.
     *        Output is in raw-count space, ready for Gauss conversion.
     */
    void applyCalibration(int16_t rx, int16_t ry, int16_t rz,
                          float& cx, float& cy, float& cz) const;

    // ── Owned members ─────────────────────────────────────────────────────────

    QMC5883L&      _sensor;
    MagDataLogger& _logger;

    MagHardIron   _hard_iron;   // current hard-iron offsets
    MagSoftIron   _soft_iron;   // current soft-iron matrix

    float         _temp_offset; // OBC thermal calibration offset (°C)
                                // offset = actual_temp - raw_reading

    MagConversion _converter;   // counts → Gauss, heading

    MagMovingAverage _filter_x;    // one filter per axis — independent
    MagMovingAverage _filter_y;
    MagMovingAverage _filter_z;

    MagSample     _latest;      // last fully-processed sample (CONVERTED mode)

    uint32_t      _sample_count;
    uint32_t      _skipped_count;
    uint32_t      _overflow_count;
};

#endif