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
/*
task() called every loop
│
├─ _project_complete? → return immediately
│
├─ compute_phase(elapsed_s())
│
├─ phase == DONE? (first time)
│   └─ set _mission_time_done = true, log message
│
├─ _mission_time_done?
│   ├─ drain_backlog_one_packet()   ← BLACKOUT → POST → PRE order
│   ├─ all_rings_empty()?
│   │   ├─ log final TX stats
│   │   ├─ shutdown()              ← sleep Iridium modem
│   │   └─ _project_complete = true
│   ├─ mission_clock::maybe_save()
│   └─ return
│
├─ _current_phase = phase_now
├─ sensor_manager::task()          ← poll all sensors
├─ sensor_manager::fill_record()   ← pack into Record struct
├─ _counter++
├─ enqueue_record_for_phase()      ← route to correct ring
├─ try_transmit_one_packet()       ← attempt Iridium TX
└─ mission_clock::maybe_save()     ← persist clock state
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
#include "modules/ls_acquisition.hpp"
#include "modules/ls_conversion.hpp"
#include "ina260.hpp"

namespace sensor_manager
{

/* ============================================================
   init()
   Call once from system_init::init() (before mission_manager::init()).
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