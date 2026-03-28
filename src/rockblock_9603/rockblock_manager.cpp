#include "rockblock_manager.hpp"
#include "iridium_driver.hpp"
#include "log_format.hpp"
#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>

namespace rockblock_manager
{

// ======================================================
// Time helper
// ======================================================

static uint32_t monotonic_ms()
{
    return to_ms_since_boot(get_absolute_time());
}

// ======================================================
// Internal state
// ======================================================

static bool         _initialized        = false;
static bool         _modem_sleeping     = false;
static ManagerState _state              = ManagerState::IDLE;
static ManagerError _last_error         = ManagerError::OK;
static uint8_t      _last_session_result= 0;
static uint32_t     _tx_interval_ms     = DEFAULT_TX_INTERVAL_MS;
static uint32_t     _last_tx_ms         = 0;

// MT message buffer
static uint8_t  _mt_buffer[iridium_driver::MAX_SBD_PAYLOAD];
static uint16_t _mt_length    = 0;
static bool     _mt_available = false;

// Payload scratch buffer
static uint8_t  _payload[iridium_driver::MAX_SBD_PAYLOAD];
static uint16_t _payload_size = 0;

// ======================================================
// Internal: transmit whatever is in _payload
// ======================================================

static bool transmit_payload()
{
    if (_payload_size == 0)
        return true;

    // ── Wake modem if sleeping ────────────────────────────
    if (_modem_sleeping)
    {
        if (!iridium_driver::wake())
        {
            _last_error = ManagerError::TRANSMIT_FAILED;
            return false;
        }
        _modem_sleeping = false;
    }

    // ── Write message to modem MO buffer ─────────────────
    if (!iridium_driver::write_message(_payload, _payload_size))
    {
        iridium_driver::sleep();
        _modem_sleeping = true;
        _last_error     = ManagerError::TRANSMIT_FAILED;
        return false;
    }

    // ── Start SBD session ─────────────────────────────────
    _state = ManagerState::SENDING;
    iridium_driver::SessionResult result = iridium_driver::start_session();
    _last_session_result = (uint8_t)result;

    if (result != iridium_driver::SessionResult::SUCCESS &&
        result != iridium_driver::SessionResult::SUCCESS_MT_TOO_BIG &&
        result != iridium_driver::SessionResult::SUCCESS_LOCATION_REJECTED)
    {
        iridium_driver::sleep();
        _modem_sleeping = true;
        _last_error     = ManagerError::SESSION_FAILED;
        _state          = ManagerState::SLEEPING;
        return false;
    }

    // ── Check for incoming MT message ─────────────────────
    if (iridium_driver::message_available())
    {
        uint16_t received = 0;
        if (iridium_driver::read_message(
                _mt_buffer,
                sizeof(_mt_buffer),
                &received))
        {
            _mt_length    = received;
            _mt_available = true;
        }
    }

    // ── Sleep modem after session ─────────────────────────
    iridium_driver::sleep();
    _modem_sleeping = true;

    _last_tx_ms = monotonic_ms();
    _last_error = ManagerError::OK;
    _state      = ManagerState::SLEEPING;

    return true;
}

// ======================================================
// Init
// ======================================================

bool init(const iridium_driver::Config& cfg)
{
    if (!iridium_driver::init(cfg))
    {
        _last_error = ManagerError::DRIVER_INIT_FAILED;
        return false;
    }

    _modem_sleeping      = false;
    _mt_available        = false;
    _mt_length           = 0;
    _payload_size        = 0;
    _last_tx_ms          = 0;
    _last_session_result = 0;
    _state               = ManagerState::IDLE;
    _last_error          = ManagerError::OK;
    _initialized         = true;

    return true;
}

bool is_initialized()
{
    return _initialized;
}

void shutdown()
{
    if (!_initialized) return;

    iridium_driver::sleep();
    _modem_sleeping = true;
    _initialized    = false;
    _state          = ManagerState::IDLE;
}

// ======================================================
// Transmission
// ======================================================

bool transmit_record(const log_format::Record& rec)
{
    if (!_initialized)
    {
        _last_error = ManagerError::NOT_INITIALIZED;
        return false;
    }

    // Pack record into payload buffer
    _payload_size = (uint16_t)log_format::RECORD_SIZE;
    memcpy(_payload, &rec, _payload_size);

    return transmit_payload();
}

bool force_transmit(const log_format::Record& rec)
{
    // Bypasses interval check — transmits immediately
    return transmit_record(rec);
}

// ======================================================
// Task — call every loop iteration
// ======================================================

bool task(const log_format::Record& rec)
{
    if (!_initialized) return false;

    uint32_t now = monotonic_ms();

    // First transmission — _last_tx_ms == 0
    bool first_tx = (_last_tx_ms == 0);

    // Interval elapsed since last successful TX
    bool interval_elapsed = (now - _last_tx_ms >= _tx_interval_ms);

    if (!first_tx && !interval_elapsed)
        return true;   // not time yet — not an error

    return transmit_record(rec);
}

void set_tx_interval_ms(uint32_t ms)
{
    _tx_interval_ms = ms;
}

// ======================================================
// Status
// ======================================================

bool         busy()                { return iridium_driver::is_busy(); }
ManagerState state()               { return _state; }
uint32_t     last_tx_ms()          { return _last_tx_ms; }
uint8_t      last_session_result() { return _last_session_result; }

// ======================================================
// MT message handling
// ======================================================

bool command_available()
{
    return _mt_available;
}

bool read_command(uint8_t*  buffer,
                  uint16_t  max_length,
                  uint16_t* received)
{
    if (!_mt_available)        return false;
    if (_mt_length > max_length) return false;

    memcpy(buffer, _mt_buffer, _mt_length);
    *received     = _mt_length;
    _mt_available = false;
    return true;
}

// ======================================================
// Error handling
// ======================================================

ManagerError last_error() { return _last_error; }

const char* error_string(ManagerError err)
{
    switch (err)
    {
        case ManagerError::OK:                 return "OK";
        case ManagerError::NOT_INITIALIZED:    return "Not initialized";
        case ManagerError::DRIVER_INIT_FAILED: return "Driver init failed";
        case ManagerError::TRANSMIT_FAILED:    return "Transmit failed";
        case ManagerError::SESSION_FAILED:     return "Session failed";
        default:                               return "Unknown";
    }
}

} // namespace rockblock_manager