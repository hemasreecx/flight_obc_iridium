#include "rockblock_manager.hpp"
#include "iridium_driver.hpp"
#include "log_format.hpp"
#include "pico/stdlib.h"
#include <string.h>
#include <stdio.h>

namespace rockblock_manager
{

// ======================================================
// Retry config
// ======================================================

static constexpr uint8_t  TX_MAX_RETRIES       = 3;
static constexpr uint32_t TX_RETRY_DELAY_MS    = 2000;
static constexpr uint32_t WAKE_SETTLE_MS       = 2000;
static constexpr uint32_t WRITE_RETRY_DELAY_MS = 1000;
static constexpr uint32_t BOOT_DELAY_MS        = 5000;

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

static bool         _initialized         = false;
static bool         _modem_sleeping      = false;
static ManagerState _state               = ManagerState::IDLE;
static ManagerError _last_error          = ManagerError::OK;
static uint8_t      _last_session_result = 0;
static uint32_t     _tx_interval_ms      = DEFAULT_TX_INTERVAL_MS;
static uint32_t     _last_tx_ms          = 0;

static uint8_t  _mt_buffer[iridium_driver::MAX_SBD_PAYLOAD];
static uint16_t _mt_length    = 0;
static bool     _mt_available = false;

static uint8_t  _payload[iridium_driver::MAX_SBD_PAYLOAD];
static uint16_t _payload_size = 0;

static log_format::Record _record_buffer[MAX_RECORDS_PER_PACKET];
static uint8_t            _buffer_count = 0;

// ======================================================
// wake_and_verify
// ======================================================

static bool wake_and_verify()
{
    if (_modem_sleeping)
    {
        if (!iridium_driver::wake())
        {
            printf("[rbmgr] wake() failed\n");
            return false;
        }
        _modem_sleeping = false;
        sleep_ms(WAKE_SETTLE_MS);
    }

    if (!iridium_driver::is_alive())
    {
        printf("[rbmgr] modem not responding after wake\n");
        return false;
    }

    printf("[rbmgr] modem alive\n");
    return true;
}

// ======================================================
// transmit_payload
// ======================================================

static bool transmit_payload()
{
    if (_payload_size == 0)
        return true;

    for (uint8_t attempt = 1; attempt <= TX_MAX_RETRIES; attempt++)
    {
        printf("[rbmgr] TX attempt %d/%d\n", attempt, TX_MAX_RETRIES);

        if (!wake_and_verify())
        {
            printf("[rbmgr] wake failed on attempt %d\n", attempt);
            sleep_ms(TX_RETRY_DELAY_MS);
            continue;
        }

        bool write_ok = false;
        for (uint8_t w = 1; w <= 3; w++)
        {
            printf("[rbmgr] write attempt %d/3\n", w);
            if (iridium_driver::write_message(_payload, _payload_size))
            {
                write_ok = true;
                break;
            }
            printf("[rbmgr] write failed: %s\n",
                   iridium_driver::error_string(iridium_driver::last_error()));
            sleep_ms(WRITE_RETRY_DELAY_MS);
        }

        if (!write_ok)
        {
            printf("[rbmgr] write_message failed after 3 attempts\n");
            iridium_driver::sleep();
            _modem_sleeping = true;
            sleep_ms(TX_RETRY_DELAY_MS);
            continue;
        }

        _state = ManagerState::SENDING;
        iridium_driver::SessionResult result = iridium_driver::start_session();
        _last_session_result = (uint8_t)result;

        printf("[rbmgr] session result: %d\n", (int)result);

        if (result == iridium_driver::SessionResult::SUCCESS ||
            result == iridium_driver::SessionResult::SUCCESS_MT_TOO_BIG ||
            result == iridium_driver::SessionResult::SUCCESS_LOCATION_REJECTED)
        {
            if (iridium_driver::message_available())
            {
                uint16_t received = 0;
                if (iridium_driver::read_message(
                        _mt_buffer, sizeof(_mt_buffer), &received))
                {
                    _mt_length    = received;
                    _mt_available = true;
                    printf("[rbmgr] MT message received: %d bytes\n", received);
                }
            }

            iridium_driver::sleep();
            _modem_sleeping = true;
            _last_tx_ms     = monotonic_ms();
            _last_error     = ManagerError::OK;
            _state          = ManagerState::SLEEPING;
            printf("[rbmgr] TX SUCCESS on attempt %d\n", attempt);
            return true;
        }

        if (result == iridium_driver::SessionResult::RADIO_DISABLED ||
            result == iridium_driver::SessionResult::BAND_VIOLATION  ||
            result == iridium_driver::SessionResult::PLL_LOCK_FAILURE)
        {
            printf("[rbmgr] permanent failure — aborting\n");
            iridium_driver::sleep();
            _modem_sleeping = true;
            _last_error     = ManagerError::SESSION_FAILED;
            _state          = ManagerState::SLEEPING;
            _last_tx_ms     = monotonic_ms();
            return false;
        }

        printf("[rbmgr] transient failure — retrying in %lums\n",
               (unsigned long)TX_RETRY_DELAY_MS);
        iridium_driver::sleep();
        _modem_sleeping = true;
        sleep_ms(TX_RETRY_DELAY_MS);
    }

    printf("[rbmgr] all %d attempts failed\n", TX_MAX_RETRIES);
    iridium_driver::sleep();
    _modem_sleeping = true;
    _last_error     = ManagerError::SESSION_FAILED;
    _state          = ManagerState::SLEEPING;
    _last_tx_ms     = monotonic_ms();
    return false;
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
    _buffer_count        = 0;
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
// force_transmit
// ======================================================

bool force_transmit(const log_format::Record& rec)
{
    if (!_initialized)
    {
        _last_error = ManagerError::NOT_INITIALIZED;
        return false;
    }

    _payload_size = (uint16_t)log_format::RECORD_SIZE;
    memcpy(_payload, &rec, _payload_size);

    return transmit_payload();
}

// ======================================================
// Task
// ======================================================

bool task(const log_format::Record& rec)
{
    if (!_initialized) return false;

    uint32_t now = monotonic_ms();
    if (now < BOOT_DELAY_MS)
        return true;

    if (_buffer_count < MAX_RECORDS_PER_PACKET)
    {
        _record_buffer[_buffer_count++] = rec;
        printf("[rbmgr] buffered %d/%d records\n",
               _buffer_count, MAX_RECORDS_PER_PACKET);
    }

    bool buffer_full      = (_buffer_count >= MAX_RECORDS_PER_PACKET);
    bool interval_elapsed = (_last_tx_ms == 0) ||
                            (now - _last_tx_ms >= _tx_interval_ms);

    bool should_tx = buffer_full || (interval_elapsed && _buffer_count > 0);

    if (!should_tx)
        return true;

    _payload_size = (uint16_t)(_buffer_count * log_format::RECORD_SIZE);
    memcpy(_payload, _record_buffer, _payload_size);

    printf("[rbmgr] transmitting %d records (%d bytes)\n",
           _buffer_count, _payload_size);

    bool ok = transmit_payload();

    _buffer_count = 0;

    return ok;
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
    if (!_mt_available)          return false;
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