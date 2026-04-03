#pragma once
#include <stdint.h>
#include "log_format.hpp"
#include "iridium_driver.hpp"

namespace rockblock_manager
{

constexpr uint32_t DEFAULT_TX_INTERVAL_MS = 7000;

constexpr uint8_t MAX_RECORDS_PER_PACKET =
    iridium_driver::MAX_SBD_PAYLOAD / log_format::RECORD_SIZE;
// = 340 / 91 = 3  (actually 273 / 91 = 3)

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

bool force_transmit(const log_format::Record& rec);

// Called by Core 1 with exactly MAX_RECORDS_PER_PACKET records.
// Packs them into _payload and calls transmit_payload().
// Returns true on success, false on all retries exhausted.
// Packet is always dropped after this call regardless of result.
bool transmit_records(const log_format::Record* records, uint8_t count);

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