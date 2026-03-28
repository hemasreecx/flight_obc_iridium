#pragma once
#include <stdint.h>
#include "hardware/uart.h"
#include "hardware/gpio.h"

namespace iridium_driver
{

// ==============================
// Constants
// ==============================

constexpr uint16_t MAX_SBD_PAYLOAD            = 340;
constexpr uint32_t DEFAULT_SESSION_TIMEOUT_MS = 20000;
constexpr uint32_t DEFAULT_AT_TIMEOUT_MS      = 1000;
constexpr uint8_t  MIN_USABLE_SIGNAL          = 2;

// ==============================
// Configuration (Pico UART)
// ==============================

struct Config
{
    uart_inst_t* uart_inst;          // uart0 or uart1 — assigned from pin_config.hpp
    uint8_t      tx_pin;        // IRIDIUM_TX_PIN
    uint8_t      rx_pin;        // IRIDIUM_RX_PIN
    uint32_t     baud_rate;     // typically 19200

    uint8_t      sleep_pin;     // IRIDIUM_ONOFF_PIN  (GPIO14)
    uint8_t      netavb_pin;    // IRIDIUM_NETAVB_PIN (GPIO2) — network available input
    uint8_t      ri_pin;        // IRIDIUM_RI_PIN     (GPIO3) — ring indicator input
};

// ==============================
// Full +SBDIX response
// ==============================

struct SbdixResult
{
    int  mo_status;
    int  momsn;
    int  mt_status;
    int  mtmsn;
    int  mt_length;
    int  mt_queued;
    bool valid;
};

// ==============================
// Enums
// ==============================

enum class SignalQuality : uint8_t
{
    NONE      = 0,
    VERY_WEAK = 1,
    WEAK      = 2,
    GOOD      = 3,
    STRONG    = 4,
    EXCELLENT = 5,
    UNKNOWN   = 255
};

/* ============================================================
   SessionResult
   Full MO status code mapping from AT+SBDIX response.
   Source: RockBLOCK 9603 AT command reference, +SBDIX section.

   0–4   : Success
   10–19 : Network / protocol failures
   32–38 : Service / radio failures
   64–65 : Hardware failures
   ============================================================ */
enum class SessionResult : uint8_t
{
    // ── Success ──────────────────────────────────────────────
    SUCCESS                   = 0,    // MO transferred successfully
    SUCCESS_MT_TOO_BIG        = 1,    // MO ok, MT message too big to transfer
    SUCCESS_LOCATION_REJECTED = 2,    // MO ok, Location Update not accepted
    // 3–4: reserved success

    // ── Network / protocol failures ──────────────────────────
    TIMEOUT_GSS               = 10,   // call did not complete in allowed time
    MO_QUEUE_FULL             = 11,   // MO message queue at GSS is full
    TOO_MANY_SEGMENTS         = 12,   // MO message has too many segments
    SESSION_INCOMPLETE_GSS    = 13,   // GSS reported session did not complete
    INVALID_SEGMENT_SIZE      = 14,   // invalid segment size
    ACCESS_DENIED             = 15,   // access is denied
    ISU_LOCKED                = 16,   // ISU locked, cannot make SBD calls
    GATEWAY_NOT_RESPONDING    = 17,   // local session timeout
    CONNECTION_LOST           = 18,   // RF drop
    LINK_FAILURE              = 19,   // protocol error caused call termination

    // ── Service / radio failures ─────────────────────────────
    NO_NETWORK                = 32,   // no network service
    ANTENNA_FAULT             = 33,   // antenna fault
    RADIO_DISABLED            = 34,   // radio disabled (*Rn command)
    ISU_BUSY                  = 35,   // ISU busy
    TRY_LATER_REGISTRATION    = 36,   // must wait 3 min since last registration
    SBD_SERVICE_DISABLED      = 37,   // SBD service temporarily disabled
    TRY_LATER_TRAFFIC         = 38,   // traffic management period

    // ── Hardware failures ─────────────────────────────────────
    BAND_VIOLATION            = 64,   // transmit outside permitted frequency band
    PLL_LOCK_FAILURE          = 65,   // hardware error during transmit

    // ── Catch-all ─────────────────────────────────────────────
    UNKNOWN_ERROR             = 255
};

enum class DriverError : uint8_t
{
    OK = 0,
    NOT_INITIALIZED,
    UART_TIMEOUT,
    MODEM_NOT_RESPONDING,
    PAYLOAD_TOO_LARGE,
    BUFFER_WRITE_FAILED,
    SESSION_IN_PROGRESS,
    SESSION_FAILED,
    NO_MESSAGE_WAITING,
    READ_FAILED,
    SLEEP_FAILED,
    RESET_FAILED,
    UNKNOWN
};

// ==============================
// API
// ==============================

bool init(const Config& config);

bool hardware_reset();
bool software_reset();

bool is_alive();
bool is_busy();
bool ring_alert_pending();

SignalQuality signal_quality();
bool network_available();

void set_session_timeout_ms(uint32_t ms);
void set_at_timeout_ms(uint32_t ms);

bool write_message(const uint8_t* data, uint16_t length);

SbdixResult   start_session_ex();
SessionResult start_session();

bool message_available();
bool read_message(uint8_t* buffer, uint16_t max_length, uint16_t* received);
bool clear_buffers();

bool sleep();
bool wake();

bool abort_session();

DriverError last_error();
const char* error_string(DriverError err);

} // namespace iridium_driver