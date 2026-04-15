/*
          START
            ↓
     sensor_manager::init()
            ↓
   ┌────────┴────────┐
   │ sensors OK?     │
   └───────┬─────────┘
           ↓
     system continues (always)

            ↓
        LOOP START
            ↓
   sensor_manager::task()
            ↓
   ┌──────────────────────┐
   │ sensor working?      │
   └───────┬──────────────┘
           │
     YES   │   NO
           ↓
   real data   simulated data

            ↓
   fill_record(record, counter)
            ↓
   record.commit = COMPLETE
            ↓
        return record
*/

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

namespace sensor_manager
{

/* ============================================================
   init()
   Call once from mission_manager::init().
   Each enabled sensor gets 3 init attempts with reset between.
   Returns true  = all enabled sensors healthy.
   Returns false = one or more sensors degraded.
   Never halts — failed sensors use simulated values.
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
   THE BRIDGE.
   Enabled + healthy  → real value
   degraded → zero / last known
   Always sets commit = COMMIT_COMPLETE.
   ============================================================ */
void fill_record(log_format::Record& r, uint32_t counter);// here using record+ count of that record not the mission counter again!! understand that

/* ============================================================
   Status
   ============================================================ */
bool imu_healthy();
bool mag_healthy();
bool lsm_healthy();

/* ============================================================
   Calibration requests
   Call before init() — flags read during init().
   Called from mission_manager based on recalib prompt.
   ============================================================ */
void request_imu_recalib();
void request_mag_recalib();
void request_lsm_recalib();

} // namespace sensor_manager