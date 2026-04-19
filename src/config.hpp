#pragma once

// -- Build/Profile ------------------------------------------------------------
// Production default: debug prints off.
#define SYSTEM_DEBUG                 0   // 1=debug prints, 0=silent
#define SENSOR_LOG_RAW               0   // 1=raw counts, 0=converted/scaled

// -- Sampling / Buses ---------------------------------------------------------
#define SAMPLE_RATE_HZ               5
#define I2C_SPEED_HZ                 400000
#define IRIDIUM_BAUD_RATE            19200

// -- Mission Timing (seconds since boot) -------------------------------------
#define PHASE_PRE_DURATION_S         450
#define PHASE_BLACKOUT_DURATION_S    45
#define PHASE_POST_DURATION_S        405
#define PHASE_TOTAL_DURATION_S  (PHASE_PRE_DURATION_S + \
                                 PHASE_BLACKOUT_DURATION_S + \
                                 PHASE_POST_DURATION_S)

// Iridium send-attempt pacing in mission_manager.
#define IRIDIUM_TX_INTERVAL_MS       20000

// Per-phase backlog capacity in packet units.
// Effective ring capacity in records =
// MAX_PACKETS_PER_PHASE * rockblock_manager::MAX_RECORDS_PER_PACKET.
#define MAX_PACKETS_PER_PHASE        12

// -- Sensor Addresses / IDs ---------------------------------------------------
#define KX134_I2C_ADDR_LOW           0x1E
#define KX134_I2C_ADDR_HIGH          0x1F
#define KX134_WHO_AM_I_VALUE         0x46

#define QMC5883L_I2C_ADDR            0x0D 

#define INA260_I2C_ADDR              0x40

#define LSM6DSV80X_I2C_ADDR_LOW      0x6A   // SA0 = 0
#define LSM6DSV80X_I2C_ADDR_HIGH     0x6B   // SA0 = 1
#define LSM6DSV80X_WHO_AM_I_VALUE    0x73

// -- Fault Handling / Recovery ------------------------------------------------
#define MAX_COMM_ERRORS              10
#define MAX_RECOVERY_ATTEMPTS        3

// Sensor init retry policy.
#define SENSOR_INIT_RETRIES          3
#define SENSOR_INIT_RETRY_DELAY_MS   2000

// Iridium retry / pacing policy.
#define IRIDIUM_TX_MAX_RETRIES       3
#define IRIDIUM_TX_RETRY_DELAY_MS    2000
#define IRIDIUM_WAKE_SETTLE_MS       2000
#define IRIDIUM_WRITE_RETRY_DELAY_MS 1000
#define IRIDIUM_AT_MAX_RETRIES       3
#define IRIDIUM_AT_RETRY_DELAY_MS    1000
#define IRIDIUM_SESSION_MAX_RETRIES  1

// -- Calibration --------------------------------------------------------------
#define KX134_CALIB_SAMPLES          1000
#define KX134_CALIB_DELAY_MS         5
#define MAG_CALIB_SAMPLES            1000
#define MAG_CALIB_DELAY_MS           5
#define LSM6DSV80X_CALIB_SAMPLES     1000
#define LSM6DSV80X_CALIB_DELAY_MS    5
#define MAG_TEMP_OFFSET_C            32.76f
//  (applied as corrected_raw = raw - offset).
#define LSM_TEMP_OFFSET_RAW          0

// Optional serial window at boot for recalibration keypresses.
#define RECALIB_PROMPT_MS            10000

// -- Placeholder Inputs (not currently wired) --------------------------------
// GPS and thermocouple are currently not installed.
// Firmware always fills safe defaults in fill_record().
#define GPS_PLACEHOLDER_TIME_S       0
#define GPS_PLACEHOLDER_LAT          0
#define GPS_PLACEHOLDER_LON          0
#define GPS_PLACEHOLDER_ALT          0


