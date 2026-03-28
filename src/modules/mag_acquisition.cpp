#include "mag_acquisition.hpp"
#include "qmc_data_logger.hpp"
#include "pico/stdlib.h"
#include <cmath>      // atan2, M_PI
#include <cstring>    // memcpy

// ── Filter window ─────────────────────────────────────────────────────────────
// At 50 Hz ODR, 8 samples = ~160 ms of smoothing.
// Matches the IMU moving-average window convention.
static constexpr uint8_t MAG_FILTER_WINDOW = 8;

// ── Constructor ───────────────────────────────────────────────────────────────

MagAcquisition::MagAcquisition(QMC5883L& sensor, MagDataLogger& logger)
    : _sensor(sensor),
      _logger(logger),
      _hard_iron{},                        // zero offsets
      _soft_iron{},                        // identity matrix (default ctor)
      _converter(),
      _filter_x(MAG_FILTER_WINDOW),
      _filter_y(MAG_FILTER_WINDOW),
      _filter_z(MAG_FILTER_WINDOW),
      _latest{},
      _temp_offset(32.76f),                  // no offset by default
      _sample_count(0),
      _skipped_count(0),
      _overflow_count(0)
{
}

// ── Lifecycle ─────────────────────────────────────────────────────────────────

bool MagAcquisition::init(const QMC5883L_Config& config)
{
    if (!_sensor.init())
        return false;

    if (!_sensor.configure(config))
        return false;

    resetCounters();
    resetFilter();

    _latest = {};    // clear any stale sample data

    return true;
}

QMC5883L_Status MagAcquisition::task()
{
    if (!_sensor.isDataReady())
        return QMC5883L_Status::NOT_READY;   // nothing to do this cycle

    if (!readAndProcess())
        return QMC5883L_Status::READ_ERROR;

    // ── Log every sample to DataLogger ────────────────────────────────────────
    // Mirror IMU pattern: logger receives the fully-processed struct,
    // caller decides what to do with overflow/skipped flags upstream.
    _logger.log(_latest);

    return QMC5883L_Status::OK;
}

// ── Calibration setters ───────────────────────────────────────────────────────

void MagAcquisition::setHardIronOffsets(int16_t x, int16_t y, int16_t z)
{
    _hard_iron.x = x;
    _hard_iron.y = y;
    _hard_iron.z = z;

    // Flush stale pre-calibration samples from all three ring buffers.
    // Without this, the filter output would blend old (uncalibrated)
    // and new (calibrated) samples for the next WINDOW cycles.
    resetFilter();
}

void MagAcquisition::setSoftIronMatrix(const float m[9])
{
    memcpy(_soft_iron.m, m, sizeof(_soft_iron.m));
    resetFilter();   // same reason as above
}

// ── Temperature offset ────────────────────────────────────────────────────────

void MagAcquisition::setTempOffset(float offset_c)
{
    // The QMC5883L temperature register is not factory calibrated.
    // Measure actual board temperature with a thermal camera or reference
    // thermometer, then compute:
    //   offset = actual_temp - sensor_raw_reading
    // Example: thermal camera reads 30.5°C, sensor reads -2.26°C
    //   offset = 30.5 - (-2.26) = 32.76°C
    _temp_offset = offset_c;
}

// ── Filter reset ──────────────────────────────────────────────────────────────

void MagAcquisition::resetFilter()
{
    _filter_x.reset();
    _filter_y.reset();
    _filter_z.reset();
}

// ── Getters ───────────────────────────────────────────────────────────────────

const MagSample& MagAcquisition::getLatestSample() const
{
    return _latest;
}

float MagAcquisition::getHeadingDeg() const
{
    return _latest.heading_deg;
}

float MagAcquisition::getTemperature() const
{
    return _latest.temperature;
}

uint32_t MagAcquisition::getSampleCount()   const { return _sample_count;   }
uint32_t MagAcquisition::getSkippedCount()  const { return _skipped_count;  }
uint32_t MagAcquisition::getOverflowCount() const { return _overflow_count; }

void MagAcquisition::resetCounters()
{
    _sample_count   = 0;
    _skipped_count  = 0;
    _overflow_count = 0;
}

// ── Private: readAndProcess() ─────────────────────────────────────────────────

bool MagAcquisition::readAndProcess()
{
    // ── 1. Read raw counts from sensor ────────────────────────────────────────
    QMC5883L_Raw raw;
    if (!_sensor.readRaw(raw))
        return false;

    // ── 2. Apply hard-iron + soft-iron calibration ────────────────────────────
    float cx, cy, cz;
    applyCalibration(raw.x, raw.y, raw.z, cx, cy, cz);

    // ── 3. Convert calibrated counts → Gauss ──────────────────────────────────
    float gx = _converter.toGauss(cx);
    float gy = _converter.toGauss(cy);
    float gz = _converter.toGauss(cz);

    // ── 4. Push each axis through its own moving-average filter ───────────────
    // Each axis is filtered independently, same as IMU, so noise on
    // one axis does not corrupt the others.
    float fx = _filter_x.update(gx);
    float fy = _filter_y.update(gy);
    float fz = _filter_z.update(gz);

    // ── 5. Compute compass heading from filtered XY values ────────────────────
    // atan2 returns radians in [-π, +π]; we shift to [0°, 360°].
    float heading = atan2f(fy, fx) * (180.0f / static_cast<float>(M_PI));
    if (heading < 0.0f)
        heading += 360.0f;

    // ── 6. Read temperature + apply OBC offset ────────────────────────────────
    // Raw sensor reading is uncalibrated — _temp_offset corrects it to
    // actual board temperature measured via thermal camera at boot.
    // offset = actual_temp - raw_reading  (e.g. 30.5 - (-2.26) = 32.76)
    float temp = 0.0f;
    _sensor.readTemperature(temp);
    temp += _temp_offset;    // apply thermal camera calibration offset

    // ── 7. Read status flags ──────────────────────────────────────────────────
    bool overflow     = _sensor.hasOverflow();
    bool data_skipped = _sensor.hasDataSkipped();

    if (overflow)     _overflow_count++;
    if (data_skipped) _skipped_count++;
    _sample_count++;

    // ── 8. Pack into _latest ──────────────────────────────────────────────────
    _latest.x_gauss      = fx;
    _latest.y_gauss      = fy;
    _latest.z_gauss      = fz;
    _latest.heading_deg  = heading;
    _latest.temperature  = temp;
    _latest.overflow     = overflow;
    _latest.data_skipped = data_skipped;
    _latest.timestamp_ms = to_ms_since_boot(get_absolute_time());

    return true;
}

// ── Private: applyCalibration() ───────────────────────────────────────────────

void MagAcquisition::applyCalibration(int16_t rx, int16_t ry, int16_t rz,
                                       float&  cx, float&  cy, float&  cz) const
{
    // ── Stage 1: Hard-iron — subtract per-axis bias ───────────────────────────
    // These offsets represent a constant magnetic field added to every
    // reading by nearby permanent magnets (motors, batteries, PCB traces).
    // Example: if the sensor always reads +200 on X with no external field,
    // set hard_iron.x = 200 and this subtracts it out.
    float hx = static_cast<float>(rx - _hard_iron.x);
    float hy = static_cast<float>(ry - _hard_iron.y);
    float hz = static_cast<float>(rz - _hard_iron.z);

    // ── Stage 2: Soft-iron — multiply by correction matrix ────────────────────
    // Soft magnetic materials (steel chassis, PCB copper planes) distort
    // the field shape from a sphere into an ellipsoid. The 3×3 matrix
    // stretches/rotates it back to a sphere.
    //
    // corrected = W * (raw - hard_iron)
    //
    //  | cx |   | m[0] m[1] m[2] |   | hx |
    //  | cy | = | m[3] m[4] m[5] | × | hy |
    //  | cz |   | m[6] m[7] m[8] |   | hz |
    //
    // When W = identity (default), this is a no-op and cx==hx etc.
    const float* m = _soft_iron.m;
    cx = m[0]*hx + m[1]*hy + m[2]*hz;
    cy = m[3]*hx + m[4]*hy + m[5]*hz;
    cz = m[6]*hx + m[7]*hy + m[8]*hz;
}