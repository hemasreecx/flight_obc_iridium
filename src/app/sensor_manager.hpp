#pragma once
#include <stdint.h>
#include "config.hpp"
#include "log_format.hpp"

#include "kx134.hpp"
#include "modules/imu_acquisition.hpp"
#include "modules/imu_conversion.hpp"
#include "logging/kx_data_logger.hpp"   // CalibStore lives here

#include "qmc5883l.hpp"
#include "modules/mag_acquisition.hpp"
#include "modules/mag_conversion.hpp"
#include "logging/qmc_data_logger.hpp"  // MagCalibStore + MagSample live here
#include "modules/ls_acquisition.hpp"
#include "modules/ls_conversion.hpp"
#include "ina260.hpp"
#include "orionb16.hpp"

namespace sensor_manager
{

/* ============================================================
   init()
   Call once from system_init::init() (before mission_manager::init()).
   Each enabled sensor gets 3 init attempts with reset between.
   Returns true  = all enabled sensors healthy.
   Returns false = one or more sensors degraded.
   Never halts — failed sensors are isolated and record fields fall back to zeros.
   ============================================================ */
bool init();

/* ============================================================
   task()
   Call every loop from mission_manager::task().
   Runs each enabled sensor acquisition.
   Handles runtime recovery — never halts.
   ============================================================ */
void task();

/* ============================================================
   fill_record()
   Packs one output record.
   - `counter` is mission time in seconds (passed from mission_manager)
   - Healthy sensors fill scaled integer fields
   - Unhealthy/no-sample sensors leave fields at zero defaults
   - Always sets commit = COMMIT_COMPLETE
   ============================================================ */
void fill_record(log_format::Record& r, uint32_t counter);

/* ============================================================
   Status
   ============================================================ */
bool imu_healthy();
bool mag_healthy();
bool lsm_healthy();
bool ina_healthy();

/* ============================================================
   Calibration requests
   Call before init() — flags read during init().
   Called from system_init::poll_sensor_recalib_requests() (per-sensor keys at boot).
   ============================================================ */
void request_imu_recalib();
void request_mag_recalib();
void request_lsm_recalib();

} // namespace sensor_manager