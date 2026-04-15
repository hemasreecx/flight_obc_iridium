#include "../modules/mag_acquisition.hpp"
#include "../logging/qmc_data_logger.hpp"
#include "pico/stdlib.h"
#include <cmath>      // atan2f, M_PI
#include <cstring>    // memcpy

// ── Filter window ─────────────────────────────────────────────────────────────
// At 50 Hz ODR, 8 samples = ~160 ms of smoothing.
static constexpr uint8_t MAG_FILTER_WINDOW = 8;

// ── Constructor ───────────────────────────────────────────────────────────────

MagAcquisition::MagAcquisition(QMC5883L& sensor, MagDataLogger& logger)
    : _sensor(sensor),
      _logger(logger),
      _hard_iron{},
      _soft_iron{},
      _converter(),
      _filter_x(MAG_FILTER_WINDOW),
      _filter_y(MAG_FILTER_WINDOW),
      _filter_z(MAG_FILTER_WINDOW),
      _latest{},
      _temp_offset(0.0f),       // set via setTempOffset() from main.cpp
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

    _latest = {};
    return true;
}

// ── task() ────────────────────────────────────────────────────────────────────
//
// Three-way behaviour based on logger mode:
//
//  PRE_CALIB  → read raw counts → logRaw() → return
//               No calibration, no conversion.
//               Use to collect min/max for hard-iron offset calculation.
//
//  POST_CALIB → read raw → apply hard-iron → read temp → logCalibrated() → return
//               No Gauss conversion, no filter.
//               cal values should center at 0 after good calibration.
//               Temperature included for OBC monitoring.
//
//  CONVERTED  → full pipeline → log() → return
//               Calibrate → Gauss → filter → heading → temperature → log.
//
QMC5883L_Status MagAcquisition::task()
{
    if (!_sensor.isDataReady())
        return QMC5883L_Status::NOT_READY;

    QMC5883L_Raw raw;
    if (!_sensor.readRaw(raw))
        return QMC5883L_Status::READ_ERROR;

    // ── PRE_CALIB — exit after raw read ───────────────────────────────────────
    // Raw int16 counts, no processing.
    // Rotate sensor in figure-8, collect min/max.
    // Compute: offset = (max + min) / 2 per axis
    if (_logger.getMode() == MagLoggerMode::PRE_CALIB)
    {
        _logger.logRaw(raw.x, raw.y, raw.z);
        _sample_count++;
        return QMC5883L_Status::OK;
    }

    // ── Apply hard-iron + soft-iron calibration ────────────────────────────────
    float cx, cy, cz;
    applyCalibration(raw.x, raw.y, raw.z, cx, cy, cz);

    // ── POST_CALIB — exit after hard-iron subtraction ─────────────────────────
    // cx/cy/cz are raw count units, centered around 0 after calibration.
    // Temperature read here so OBC thermal state is visible at this stage.
    if (_logger.getMode() == MagLoggerMode::POST_CALIB)
    {
        float temp = 0.0f;
        _sensor.readTemperature(temp);
        temp += _temp_offset;

        _logger.logCalibrated(cx, cy, cz, temp);
        _sample_count++;
        return QMC5883L_Status::OK;
    }

    // ── CONVERTED — full pipeline ─────────────────────────────────────────────
    if (!readAndProcess(raw))
        return QMC5883L_Status::READ_ERROR;

    _logger.log(_latest);
    return QMC5883L_Status::OK;
}

// ── Calibration setters ───────────────────────────────────────────────────────

void MagAcquisition::setHardIronOffsets(int16_t x, int16_t y, int16_t z)
{
    _hard_iron.x = x;
    _hard_iron.y = y;
    _hard_iron.z = z;
    resetFilter();
}

void MagAcquisition::setSoftIronMatrix(const float m[9])
{
    memcpy(_soft_iron.m, m, sizeof(_soft_iron.m));
    resetFilter();
}

void MagAcquisition::setTempOffset(float offset_c)
{
    // The QMC5883L temperature register is not factory calibrated.
    // Measure actual board temp with a thermal camera, then compute:
    //   offset = actual_temp - sensor_raw_reading
    // Example: thermal camera = 30.5°C, sensor reads -2.26°C
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

const MagSample& MagAcquisition::getLatestSample() const { return _latest;            }
float    MagAcquisition::getHeadingDeg()    const        { return _latest.heading_deg; }
float    MagAcquisition::getTemperature()   const        { return _latest.temperature; }
uint32_t MagAcquisition::getSampleCount()   const        { return _sample_count;       }
uint32_t MagAcquisition::getSkippedCount()  const        { return _skipped_count;      }
uint32_t MagAcquisition::getOverflowCount() const        { return _overflow_count;     }

void MagAcquisition::resetCounters()
{
    _sample_count   = 0;
    _skipped_count  = 0;
    _overflow_count = 0;
}

// ── Private: readAndProcess() — CONVERTED mode only ───────────────────────────

bool MagAcquisition::readAndProcess(const QMC5883L_Raw& raw)
{
    // ── 1. Apply hard-iron + soft-iron calibration ────────────────────────────
    float cx, cy, cz;
    applyCalibration(raw.x, raw.y, raw.z, cx, cy, cz);

    // ── 2. Convert calibrated counts → Gauss ──────────────────────────────────
    // ±8G range: 3000 LSB per Gauss → divide by 3000
    float gx = _converter.toGauss(cx);
    float gy = _converter.toGauss(cy);
    float gz = _converter.toGauss(cz);

    // ── 3. Moving-average filter — one per axis ───────────────────────────────
    // Window = 8 samples at 50Hz = ~160ms smoothing.
    // Each axis filtered independently so noise on one axis
    // does not corrupt the others.
    float fx = _filter_x.update(gx);
    float fy = _filter_y.update(gy);
    float fz = _filter_z.update(gz);

    // ── 4. Compass heading from filtered XY ───────────────────────────────────
    // atan2(y, x) gives angle from +X axis in [-π, +π] radians.
    // Shifted to [0°, 360°] for compass convention.
    // NOTE: assumes sensor mounted flat (XY plane = horizontal).
    // For tilted OBC mounting, tilt compensation using KX134
    // accelerometer roll/pitch angles is needed for accuracy.
    float heading = atan2f(fy, fx) * (180.0f / static_cast<float>(M_PI));
    if (heading < 0.0f)
        heading += 360.0f;

    // ── 5. Temperature + OBC offset ───────────────────────────────────────────
    // Raw reading is uncalibrated. _temp_offset corrects to actual board temp.
    // offset = actual_temp(thermal camera) - raw_reading
    float temp = 0.0f;
    if (!_sensor.readTemperature(temp))
        return false;
    temp += _temp_offset;

    // ── 6. Status flags ───────────────────────────────────────────────────────
    bool overflow     = _sensor.hasOverflow();
    bool data_skipped = _sensor.hasDataSkipped();

    if (overflow)     _overflow_count++;
    if (data_skipped) _skipped_count++;
    _sample_count++;

    // ── 7. Pack into _latest ──────────────────────────────────────────────────
    _latest.raw_x        = raw.x;
    _latest.raw_y        = raw.y;
    _latest.raw_z        = raw.z;
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
    // Stage 1: Hard-iron — subtract per-axis constant bias
    // These offsets represent constant magnetic field from nearby
    // permanent magnets (motors, batteries, PCB traces).
    float hx = static_cast<float>(rx - _hard_iron.x);
    float hy = static_cast<float>(ry - _hard_iron.y);
    float hz = static_cast<float>(rz - _hard_iron.z);

    // Stage 2: Soft-iron — multiply by correction matrix
    // Corrects field shape from ellipsoid back to sphere.
    // corrected = W * (raw - hard_iron)
    // When W = identity (default): cx==hx, cy==hy, cz==hz
    const float* m = _soft_iron.m;
    cx = m[0]*hx + m[1]*hy + m[2]*hz;
    cy = m[3]*hx + m[4]*hy + m[5]*hz;
    cz = m[6]*hx + m[7]*hy + m[8]*hz;
}