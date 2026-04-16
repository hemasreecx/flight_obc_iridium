#pragma once
#include <stdint.h>
#include "config.hpp"
#include "log_format.hpp"

#include "iridium_driver.hpp"
#include "rockblock_manager.hpp"

namespace mission_manager
{

/* ============================================================
   Mission phases — time based
   PRE      : 0 → PHASE_PRE_DURATION_S
   BLACKOUT : PRE_END → PRE_END + PHASE_BLACKOUT_DURATION_S
   POST     : remainder until power off
   ============================================================ */
enum class Phase : uint8_t
{
    NONE     = 0,
    PRE      = 1,
    BLACKOUT = 2,
    POST     = 3,
    DONE     = 4
};

/* ============================================================
   init()
   Call from system_init::init() after sensor_manager and Iridium init.
   Resets mission buffers/counters and starts mission_clock.
   Returns false if mission_clock::init() fails.
   ============================================================ */
bool init();

/* ============================================================
   task()
   Call every loop from system_init/main runtime loop.
   - Computes mission phase from mission_clock
   - Runs sensor_manager::task() and fills one record
   - Enqueues into the current phase ring
   - Attempts one Iridium packet transmit per loop
   - On mission end: drains BLACKOUT → POST → PRE, then marks complete
   ============================================================ */
void task();

/* ============================================================
   shutdown()
   Puts Iridium modem into sleep state.
   ============================================================ */
void shutdown();

/* ============================================================
   Status
   ============================================================ */
Phase       current_phase();
uint32_t    elapsed_seconds();
uint32_t    record_counter();  // loop record count (separate from mission timestamp)
bool        mission_complete();
uint32_t    transmitted_record_count();
uint32_t    transmitted_packet_count();
uint32_t    dropped_record_count();
uint32_t    dropped_packet_count();

} // namespace mission_manager