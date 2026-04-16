/*  
init()
   ↓
loop:
   sensor_manager::task()
   ↓
   fill_record() - in ring buffer 20 samples per sec .. can create 3 ring buffers for 3 phases
   ↓
   check phase
   ↓
   transmit (Iridium)
   ↓
   debug print

*/

// currently this code is changing file sbased ontime not on our logic of first pre then when time comes then to blackout..
//  after drain out to post

#include "mission_manager.hpp"
#include "mission_clock.hpp"
#include "sensor_manager.hpp"
#include "pin_config.hpp"
#include "config.hpp"
#include "log_format.hpp"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdint.h>

#include "iridium_driver.hpp"
#include "rockblock_manager.hpp"

#if SYSTEM_DEBUG
#define DBG(...) do { printf(__VA_ARGS__); fflush(stdout); } while(0)
#else
#define DBG(...)
#endif

namespace mission_manager
{

// ============================================================
// Internal state // are we storing these in Flash or just for on the go?
// ============================================================

static Phase    _current_phase = Phase::NONE;
static uint32_t _counter       = 0;

static log_format::Record _latest_record = {};

// ============================================================
// Time helpers
// ============================================================

static uint32_t elapsed_s()
{
    return mission::mission_clock::now_seconds();
}

// ============================================================
// Phase helpers
// ============================================================

static Phase compute_phase(uint32_t secs)
{
    if (secs < PHASE_PRE_DURATION_S)
        return Phase::PRE;

    if (secs < (PHASE_PRE_DURATION_S + PHASE_BLACKOUT_DURATION_S))
        return Phase::BLACKOUT;

    if (secs < PHASE_TOTAL_DURATION_S)
        return Phase::POST;

    return Phase::DONE;
}

static const char* phase_name(Phase p)
{
    switch (p)
    {
        case Phase::PRE:      return "PRE";
        case Phase::BLACKOUT: return "BLACKOUT";
        case Phase::POST:     return "POST";
        default:              return "NONE";
    }
}



// ============================================================
// Phase switch
// ============================================================

static void switch_phase(Phase new_phase)
{
    DBG("[mission] phase %s → %s at %lus\n",
        phase_name(_current_phase),
        phase_name(new_phase),
        elapsed_s());

    _current_phase = new_phase;
}

// ============================================================
// USB debug print
// ============================================================

static uint32_t _print_counter = 0;
static constexpr uint32_t PRINT_EVERY = 1;

static void print_record(const log_format::Record& r)
{
    if ((_print_counter++ % PRINT_EVERY) != 0)
        return;

    printf("[%s] ctr=%lu "
           "acc=%d,%d,%d "
           "mag=%d,%d,%d "
           "temp=%.2f "
           "bat=%umV\n",
           phase_name(_current_phase),
           r.counter,
           r.acc3_x, r.acc3_y, r.acc3_z,
           r.mag_x,  r.mag_y,  r.mag_z,
           (double)(r.obc_temperature / 100.0f),
           r.battery_voltage);
}
static void handle_iridium()
{
    static uint32_t last_tx_ms = 0;
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if ((now_ms - last_tx_ms) < IRIDIUM_TX_INTERVAL_MS)
        return;

    last_tx_ms = now_ms;
    bool tx_ok = rockblock_manager::force_transmit(_latest_record);
    if (!tx_ok)
    {
        DBG("[mission] Iridium live TX failed\n");
    }
}
// ============================================================
// Recalibration prompt
// ============================================================

static bool wait_for_recalib_request()
{
    printf("Send 'c' within %d seconds to recalibrate...\n",
           RECALIB_PROMPT_MS / 1000);
    fflush(stdout);

    uint32_t deadline = to_ms_since_boot(get_absolute_time())
                        + RECALIB_PROMPT_MS;

    while (to_ms_since_boot(get_absolute_time()) < deadline)
    {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT &&
            (ch == 'c' || ch == 'C'))
        {
            printf("Recalibration requested.\n");
            fflush(stdout);
            return true;
        }
        sleep_ms(10);
    }
    return false;
}

// ============================================================
// init()
// ============================================================

bool init()
{
    _current_phase = Phase::NONE;
    _counter       = 0;

    mission::mission_clock::init();

    DBG("[mission] ========================\n");
    DBG("[mission] OBC FLIGHT SYSTEM BOOT\n");
    DBG("[mission] ========================\n");

    // ── Recalibration prompt ──────────────────────────────────
    bool force_recalib = wait_for_recalib_request();
    if (force_recalib)
    {
        sensor_manager::request_imu_recalib();
        sensor_manager::request_mag_recalib();
        sensor_manager::request_lsm_recalib();
        sensor_manager::request_lsm_recalib();
    }

    // ── Sensor init ───────────────────────────────────────────
    bool sensors_ok = sensor_manager::init();
    if (!sensors_ok)
        DBG("[mission] WARN: one or more sensors degraded\n");
    else
        DBG("[mission] all sensors OK\n");

    iridium_driver::Config iridium_cfg;
    iridium_cfg.uart_inst  = IRIDIUM_UART;
    iridium_cfg.tx_pin     = IRIDIUM_TX_PIN;
    iridium_cfg.rx_pin     = IRIDIUM_RX_PIN;
    iridium_cfg.baud_rate  = 19200;
    iridium_cfg.sleep_pin  = IRIDIUM_ONOFF_PIN;
    iridium_cfg.netavb_pin = IRIDIUM_NETAVB_PIN;
    iridium_cfg.ri_pin     = IRIDIUM_RI_PIN;

    if (!rockblock_manager::init(iridium_cfg))
    {
        DBG("[mission] WARN: Iridium init failed — %s\n",
            rockblock_manager::error_string(
                rockblock_manager::last_error()));
    }
    else
    {
        DBG("[mission] Iridium OK\n");
    }

    // ── Open PRE phase — always start here ───────────────────
    switch_phase(Phase::PRE);

    DBG("[mission] PRE:      0 → %ds\n",
        PHASE_PRE_DURATION_S);
    DBG("[mission] BLACKOUT: %d → %ds\n",
        PHASE_PRE_DURATION_S,
        PHASE_PRE_DURATION_S + PHASE_BLACKOUT_DURATION_S);
    DBG("[mission] POST:     %ds → %ds\n",
    PHASE_PRE_DURATION_S + PHASE_BLACKOUT_DURATION_S,
    PHASE_TOTAL_DURATION_S);
    DBG("[mission] flight loop starting\n");

    return true;
}

void task()
{
    DBG("[mission] task() start\n");

    // ── 1. Sensor tasks ───────────────────────────────────────
    sensor_manager::task();
    DBG("[mission] sensors done\n");

    // ── 2. Fill record with mission time as the record counter ──
    const uint32_t record_time = mission::mission_clock::now_seconds();
    sensor_manager::fill_record(_latest_record, record_time);
    _counter++;
    DBG("[mission] record filled\n");

    // ── 3. Phase boundary check ───────────────────────────────
    // [mission] phase PRE → BLACKOUT at 60s
    Phase new_phase = compute_phase(elapsed_s());
    if (new_phase != _current_phase)
    {
        if (new_phase == Phase::DONE)
        {
            // Mission complete — close files, sleep everything
            DBG("[mission] MISSION COMPLETE at %lus\n", elapsed_s());
            shutdown();
            return;
        }
        switch_phase(new_phase);
    }
    DBG("[mission] phase check done\n");
    handle_iridium();
    DBG("[mission] iridium done\n");
}

// ============================================================
// shutdown()
// ============================================================

void shutdown()
{
    DBG("[mission] shutdown\n");


    rockblock_manager::shutdown();


    _current_phase = Phase::NONE;
}

// ============================================================
// Status
// ============================================================

Phase    current_phase()   { return _current_phase; }
uint32_t elapsed_seconds() { return elapsed_s(); }
uint32_t record_counter()  { return _counter; } // it is counting no.of records logged

} // namespace mission_manager