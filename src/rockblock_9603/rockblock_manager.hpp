#pragma once
#include <stdint.h>
#include "log_format.hpp"
#include "iridium_driver.hpp"

namespace rockblock_manager
{

constexpr uint32_t DEFAULT_TX_INTERVAL_MS = 7000;

enum class ManagerState : uint8_t
{
    IDLE,
    SENDING,
    SLEEPING
};

enum class ManagerError : uint8_t
{
    OK = 0,
    NOT_INITIALIZED,
    DRIVER_INIT_FAILED,
    TRANSMIT_FAILED,
    SESSION_FAILED,
    UNKNOWN
};

bool init(const iridium_driver::Config& cfg);
bool is_initialized();
void shutdown();

// Pass record directly — no internal buffer
bool transmit_record(const log_format::Record& rec);
bool force_transmit(const log_format::Record& rec);


// Call every loop — handles timing + sleep/wake
bool task(const log_format::Record& rec);

void set_tx_interval_ms(uint32_t ms);

// ── Status ───────────────────────────────────────────────
bool         busy();
ManagerState state();
uint32_t     last_tx_ms();
uint8_t      last_session_result();

// ── MT message handling ───────────────────────────────────
bool command_available();
bool read_command(uint8_t*  buffer,
                  uint16_t  max_length,
                  uint16_t* received);

// ── Error handling ────────────────────────────────────────
ManagerError last_error();
const char*  error_string(ManagerError err);

} // namespace rockblock_manager