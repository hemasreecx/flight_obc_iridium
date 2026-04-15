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
   Call once from main().
   - Shows recalib prompt (FLIGHT_MODE only)
   - Inits sensor_manager
   
   - Inits Iridium
   Returns false if critical init fails.
   System runs degraded if non-critical subsystem fails.
   ============================================================ */
bool init();

/* ============================================================
   task()
   Call every loop from main().
   - Runs sensor tasks
   - stores in buffer file
   - Checks phase boundary → switches file if needed
   - Routes record to enabled outputs
   ============================================================ */
void task();

/* ============================================================
   shutdown()
   Flush + close SD file, sleep Iridium.
   ============================================================ */
void shutdown();

/* ============================================================
   Status
   ============================================================ */
Phase       current_phase();
uint32_t    elapsed_seconds();
uint32_t    record_counter();  // increments with each record, never resets

} // namespace mission_manager