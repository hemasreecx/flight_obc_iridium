#pragma once

// ── Debug ─────────────────────────────────────────────────────
#define SYSTEM_DEBUG            1   // 1=debug prints, 0=silent
#define SENSOR_LOG_RAW          0   // 1=raw counts, 0=converted

// ── Sensor sample rate ────────────────────────────────────────
#define SAMPLE_RATE_HZ          5

// ── I2C speed ─────────────────────────────────────────────────
#define I2C_SPEED_HZ            400000
// ── Phase timing ──────────────────────────────────────────────
// Time-based phase switching (seconds since boot)
// PRE    : 0 → PHASE_PRE_DURATION_S
// BLACKOUT: PHASE_PRE_DURATION_S → PHASE_PRE_DURATION_S + PHASE_BLACKOUT_DURATION_S
// POST   : remainder until power off
#define PHASE_PRE_DURATION_S        450
#define PHASE_BLACKOUT_DURATION_S   45
#define PHASE_POST_DURATION_S      405   
// add this to config.hpp
#define PHASE_TOTAL_DURATION_S  (PHASE_PRE_DURATION_S + \
                                  PHASE_BLACKOUT_DURATION_S + \
                                  PHASE_POST_DURATION_S)

#define IRIDIUM_TX_INTERVAL_MS  20000

// Max packets worth of records buffered per mission phase.
// Effective ring size per phase = MAX_PACKETS_PER_PHASE * rockblock_manager::MAX_RECORDS_PER_PACKET.
#define MAX_PACKETS_PER_PHASE    8

// -- Sensor addresses ---------------------------------------------------------

#define KX134_I2C_ADDR_LOW      0x1E
#define KX134_I2C_ADDR_HIGH     0x1F
#define KX134_WHO_AM_I_VALUE    0x46
#define QMC5883L_I2C_ADDR       0x0D   // fixed
#define LSM6DSV80X_I2C_ADDR_LOW     0x6A   // SA0 = 0
#define LSM6DSV80X_I2C_ADDR_HIGH    0x6B   // SA0 = 1
#define LSM6DSV80X_WHO_AM_I_VALUE   0x73

#define MAX_COMM_ERRORS         10
#define MAX_RECOVERY_ATTEMPTS   3
#define KX134_CALIB_SAMPLES     1000
#define KX134_CALIB_DELAY_MS    5
#define MAG_CALIB_SAMPLES       1000
#define MAG_CALIB_DELAY_MS      5
#define LSM6DSV80X_CALIB_SAMPLES       1000
#define LSM6DSV80X_CALIB_DELAY_MS      5
// MAG temperature offset (deg C) applied in MagAcquisition
#define MAG_TEMP_OFFSET_C       32.76f
// Boot window (ms): optional per-sensor recalibration via USB serial — see system_init.cpp
#define RECALIB_PROMPT_MS       10000

