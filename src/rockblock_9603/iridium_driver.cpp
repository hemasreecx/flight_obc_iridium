#include "iridium_driver.hpp"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <string.h>
#include <stdio.h>

namespace iridium_driver
{

static uint64_t now_ms()
{
    return to_ms_since_boot(get_absolute_time());
}

static uart_inst_t* _uart_inst = nullptr;

static bool uart_send(const uint8_t* data, size_t len, uint32_t timeout_ms)
{
    uint64_t deadline = now_ms() + timeout_ms;
    for (size_t i = 0; i < len; i++)
    {
        while (!uart_is_writable(_uart_inst))
        {
            if (now_ms() >= deadline) return false; // timeout
            sleep_us(10);
        }
        uart_putc_raw(_uart_inst, (char)data[i]);
    }
    return true;
}

static bool uart_read_byte(uint8_t* out, uint32_t timeout_ms)
{
    uint64_t deadline = now_ms() + timeout_ms; // RX TIMEOUT
    while (now_ms() < deadline)
    {
        if (uart_is_readable(_uart_inst))
        {
            *out = (uint8_t)uart_getc(_uart_inst);
            return true;
        }
        sleep_us(100);
    }
    return false;
}

static bool uart_read_line(char* buffer, size_t max_len, uint32_t timeout_ms)
{
    if (max_len == 0) return false;
    size_t   idx      = 0;
    uint64_t deadline = now_ms() + timeout_ms;

    while (now_ms() < deadline)
    {
        uint8_t b = 0;
        if (!uart_read_byte(&b, 10)) continue;  // 10ms wait per each byte 

        char c = (char)b;
        if (idx < max_len - 1) buffer[idx++] = c;
        if (c == '\n') break;
    }

    buffer[idx] = '\0';
    return idx > 0;
}


static Config      _cfg;
static bool        _initialized     = false;
static bool        _session_active  = false;
static bool        _mt_pending      = false;
static uint32_t    _session_timeout = DEFAULT_SESSION_TIMEOUT_MS;
static uint32_t    _at_timeout      = DEFAULT_AT_TIMEOUT_MS;
static DriverError _last_error      = DriverError::OK;

static constexpr uint8_t  AT_MAX_RETRIES    = 3;
static constexpr uint32_t AT_RETRY_DELAY_MS = 1000;
static constexpr uint8_t  SES_MAX_RETRIES   = 1;

static bool port_send(const char* s)
{
    return uart_send(
        reinterpret_cast<const uint8_t*>(s),
        strlen(s),
        _at_timeout
    );
}

static bool port_read_line(char* buffer, size_t max_len, uint32_t timeout_ms)
{
    return uart_read_line(buffer, max_len, timeout_ms);
}

static bool send_at(const char* cmd)
{
    uint64_t t0 = now_ms();
    port_send(cmd);
    port_send("\r");

    char     line[128];
    uint64_t deadline = now_ms() + _at_timeout;

    while (now_ms() < deadline)
    {
        if (!port_read_line(line, sizeof(line), 100)) continue;

        if (strstr(line, "OK"))
        {
            printf("[AT] %s -> OK (%llums)\n",
                   cmd, (unsigned long long)(now_ms() - t0));
            return true;
        }
        if (strstr(line, "ERROR"))
        {
            printf("[AT] %s -> ERROR (%llums)\n",
                   cmd, (unsigned long long)(now_ms() - t0));
            _last_error = DriverError::MODEM_NOT_RESPONDING;
            return false;
        }
    }

    printf("[AT] %s -> TIMEOUT (%llums)\n",
           cmd, (unsigned long long)(now_ms() - t0));
    _last_error = DriverError::UART_TIMEOUT;
    return false;
}

static bool send_at_retry(const char* cmd,
                           uint8_t max_retries = AT_MAX_RETRIES)
{
    for (uint8_t attempt = 0; attempt < max_retries; attempt++)
    {
        if (send_at(cmd)) return true;
        if (_last_error != DriverError::UART_TIMEOUT) break; // last_error says what went wrong the last time 
        if (attempt < max_retries - 1) sleep_ms(AT_RETRY_DELAY_MS); // after two tries it will go into sleep mode
    }
    return false;
}

static void set_sleep_pin(bool awake)
{
    if (_cfg.sleep_pin != 0xFF)
        gpio_put(_cfg.sleep_pin, awake ? 1 : 0);
}


bool init(const Config& config)
{
    if (_initialized) sleep();

    _cfg       = config;
    _uart_inst = _cfg.uart_inst;

    if (_uart_inst == nullptr)
    {
        _last_error = DriverError::NOT_INITIALIZED;
        return false;
    }

printf("[iridium] raw test — sending AT every 500ms for 5s\n"); // raw loop back test or bring up test
fflush(stdout);

uint64_t test_end = now_ms() + 5000;
int cycle = 0;
while (now_ms() < test_end)
{
    printf("[cycle %d] sending: A T CR\n", cycle++);
    fflush(stdout);
    uart_putc_raw(_uart_inst, 'A');
    uart_putc_raw(_uart_inst, 'T');
    uart_putc_raw(_uart_inst, '\r');

    sleep_ms(500);

    printf("[cycle] rx: ");
    if (!uart_is_readable(_uart_inst))
    {
        printf("(nothing)");
    }
    while (uart_is_readable(_uart_inst))
    {
        uint8_t b = uart_getc(_uart_inst);
        if (b >= 32 && b < 127) printf("'%c'", b);
        else printf("[0x%02X]", b);  // o k  \n\r -> here those char are invisible so  - 'O''K'[0x0D][0x0A]
    }
    printf("\n");
    fflush(stdout);
}

    // ── Drain boot messages ─────────────────────────────────── /// removing the garbage / partial read
    sleep_ms(2000);
    uint32_t drained = 0;
    while (uart_is_readable(_uart_inst))
    {
        char c = uart_getc(_uart_inst);
        if (c >= 32 && c < 127) printf("%c", c);
        else printf("[0x%02X]", (uint8_t)c);
        drained++;
    }
    if (drained > 0) printf("\n");
    printf("[iridium] drained %lu bytes\n", drained);
    fflush(stdout);
    sleep_ms(100);

    if (_cfg.sleep_pin != 0xFF) // sleep pin 0 -> active, 1-> sleep mode
    {
        gpio_init(_cfg.sleep_pin);
        gpio_set_dir(_cfg.sleep_pin, GPIO_OUT);
        gpio_put(_cfg.sleep_pin, 0);
        sleep_ms(100);
        gpio_put(_cfg.sleep_pin, 1);
        sleep_ms(100);
        gpio_put(_cfg.sleep_pin, 0);
        sleep_ms(2000);
        // this is toggling sequence -> where we  gonna try to put the modem in a known state -> as it may be partial sleep or half powered... basically reformatting the state
    }

    // ── Optional input pins ───────────────────────────────────
    if (_cfg.netavb_pin != 0xFF)
    {
        gpio_init(_cfg.netavb_pin);
        gpio_set_dir(_cfg.netavb_pin, GPIO_IN);
    }
    if (_cfg.ri_pin != 0xFF)
    {
        gpio_init(_cfg.ri_pin);
        gpio_set_dir(_cfg.ri_pin, GPIO_IN);
    }

  // after uart clean, pins set, then again checking whether the AT command is working or not
    if (!send_at_retry("AT"))
    {
        _last_error = DriverError::MODEM_NOT_RESPONDING;
        return false;
    }

    clear_buffers();

    _session_active = false;
    _mt_pending     = false;
    _last_error     = DriverError::OK;
    _initialized    = true;
    return true;
}
// these are high level check functions 
bool is_alive()           { return send_at("AT"); }
bool is_busy()            { return _session_active; }
bool ring_alert_pending() { return _mt_pending; }


SignalQuality signal_quality()
{
    if (!_initialized)
    {
        _last_error = DriverError::NOT_INITIALIZED;
        return SignalQuality::UNKNOWN;
    }

    uint64_t t0 = now_ms();
    port_send("AT+CSQ\r");

    char     line[64];
    uint64_t deadline = now_ms() + _at_timeout;

    while (now_ms() < deadline)
    {
        if (!port_read_line(line, sizeof(line), 100)) continue;

        int v = -1;
        if (sscanf(line, "+CSQ: %d", &v) == 1 ||
            sscanf(line, "+CSQ:%d",  &v) == 1)
        {
            if (v >= 0 && v <= 5)
            {
                printf("[CSQ] signal=%d (%llums)\n",
                       v, (unsigned long long)(now_ms() - t0));
                return static_cast<SignalQuality>(v);
            }
        }
        if (strstr(line, "OK"))    break;
        if (strstr(line, "ERROR")) break;
    }

    printf("[CSQ] TIMEOUT (%llums)\n",
           (unsigned long long)(now_ms() - t0));
    _last_error = DriverError::UART_TIMEOUT;
    return SignalQuality::UNKNOWN;
}

bool network_available()
{
    return (uint8_t)signal_quality() >= MIN_USABLE_SIGNAL;
}


void set_session_timeout_ms(uint32_t ms) { _session_timeout = ms; }
void set_at_timeout_ms(uint32_t ms)      { _at_timeout = ms; }


bool write_message(const uint8_t* data, uint16_t length)
{
    if (!_initialized)
    {
        _last_error = DriverError::NOT_INITIALIZED;
        return false;
    }
    if (_session_active)
    {
        _last_error = DriverError::SESSION_IN_PROGRESS;
        return false;
    }
    if (length > MAX_SBD_PAYLOAD)
    {
        _last_error = DriverError::PAYLOAD_TOO_LARGE;
        return false;
    }

    uint64_t t0 = now_ms();

    sleep_ms(1000);
    printf("[SBDWB] sending AT+SBDWB=%d\n", (int)length);

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+SBDWB=%d", (int)length);
    port_send(cmd);
    port_send("\r");

    char     line[64];
    bool     ready    = false;
    uint64_t deadline = now_ms() + _at_timeout;

    while (now_ms() < deadline)
    {
        if (!port_read_line(line, sizeof(line), 100)) continue;
        if (strstr(line, "READY")) { ready = true; break; }
        if (strstr(line, "ERROR"))
        {
            printf("[SBDWB] ERROR waiting READY (%llums)\n",
                   (unsigned long long)(now_ms() - t0));
            _last_error = DriverError::BUFFER_WRITE_FAILED;
            return false;
        }
    }

    if (!ready)
    {
        printf("[SBDWB] TIMEOUT waiting READY (%llums)\n",
               (unsigned long long)(now_ms() - t0));
        _last_error = DriverError::UART_TIMEOUT;
        return false;
    }

    if (!uart_send(data, length, _at_timeout))
    {
        printf("[SBDWB] payload write FAILED (%llums)\n",
               (unsigned long long)(now_ms() - t0));
        _last_error = DriverError::UART_TIMEOUT;
        return false;
    }

    uint16_t checksum = 0;
    for (uint16_t i = 0; i < length; i++)
        checksum = (uint16_t)(checksum + data[i]);

    uint8_t cs[2] = { (uint8_t)(checksum >> 8),
                      (uint8_t)(checksum & 0xFF) };
    if (!uart_send(cs, sizeof(cs), _at_timeout))
    {
        printf("[SBDWB] checksum write FAILED (%llums)\n",
               (unsigned long long)(now_ms() - t0));
        _last_error = DriverError::UART_TIMEOUT;
        return false;
    }

    deadline = now_ms() + _at_timeout;
    while (now_ms() < deadline)
    {
        if (!port_read_line(line, sizeof(line), 100)) continue;
        int status = -1;
        if (sscanf(line, "%d", &status) == 1)
        {
            printf("[SBDWB] status=%d (%llums)\n",
                   status, (unsigned long long)(now_ms() - t0));
            if (status == 0) { _last_error = DriverError::OK;                  return true;  }
            if (status == 1) { _last_error = DriverError::UART_TIMEOUT;        return false; }
            if (status == 2) { _last_error = DriverError::BUFFER_WRITE_FAILED; return false; }
        }
        if (strstr(line, "ERROR"))
        {
            _last_error = DriverError::BUFFER_WRITE_FAILED;
            return false;
        }
    }

    printf("[SBDWB] TIMEOUT waiting status (%llums)\n",
           (unsigned long long)(now_ms() - t0));
    _last_error = DriverError::UART_TIMEOUT;
    return false;
}

static SbdixResult attempt_session_ex()
{
    SbdixResult res  = {};
    res.valid        = false;
    res.mo_status    = 32;

    _session_active  = true;
    _mt_pending      = false;

    uint64_t t0 = now_ms();
    printf("[SBDIX] sending AT+SBDIX (timeout=%ums)...\n",
           (unsigned)_session_timeout);

    port_send("AT+SBDIX\r");

    char     line[128];
    uint64_t deadline = now_ms() + _session_timeout;

    while (now_ms() < deadline)
    {
        if (!port_read_line(line, sizeof(line), 100)) continue;

        int parsed =
            sscanf(line, "+SBDIX: %d, %d, %d, %d, %d, %d",
                   &res.mo_status, &res.momsn,
                   &res.mt_status, &res.mtmsn,
                   &res.mt_length, &res.mt_queued);

        if (parsed < 1)
            parsed = sscanf(line, "+SBDIX:%d,%d,%d,%d,%d,%d",
                            &res.mo_status, &res.momsn,
                            &res.mt_status, &res.mtmsn,
                            &res.mt_length, &res.mt_queued);

        if (parsed >= 1)
        {
            res.valid = true;
            if (res.mt_status == 1 || res.mt_status == 2)
                _mt_pending = true;

            printf("[SBDIX] mo=%d momsn=%d mt=%d mt_len=%d "
                   "mt_q=%d fields=%d (%llums)\n",
                   res.mo_status, res.momsn, res.mt_status,
                   res.mt_length, res.mt_queued, parsed,
                   (unsigned long long)(now_ms() - t0));
            break;
        }
    }

    if (!res.valid)
    {
        printf("[SBDIX] no response — TIMEOUT (%llums)\n",
               (unsigned long long)(now_ms() - t0));
    }

    _session_active = false;
    _last_error     = (res.mo_status == 0)
                      ? DriverError::OK
                      : DriverError::SESSION_FAILED;
    return res;
}

SbdixResult start_session_ex()
{
    SbdixResult last_res  = {};
    last_res.valid        = false;
    last_res.mo_status    = (int)SessionResult::UNKNOWN_ERROR;

    if (!_initialized)
    {
        _last_error = DriverError::NOT_INITIALIZED;
        return last_res;
    }
    if (_session_active)
    {
        _last_error = DriverError::SESSION_IN_PROGRESS;
        return last_res;
    }

    for (uint8_t attempt = 0; attempt < SES_MAX_RETRIES; attempt++)
    {
        last_res = attempt_session_ex();
        if (last_res.mo_status == 0) return last_res;

        if (last_res.mo_status == 34)
        {
            _last_error = DriverError::SESSION_FAILED;
            return last_res;
        }
    }

    _last_error = DriverError::SESSION_FAILED;
    return last_res;
}

SessionResult start_session()
{
    SbdixResult res = start_session_ex();

    switch (res.mo_status)
    {
        case 0:  return SessionResult::SUCCESS;
        case 1:  return SessionResult::SUCCESS_MT_TOO_BIG;
        case 2:  return SessionResult::SUCCESS_LOCATION_REJECTED;
        case 10: return SessionResult::TIMEOUT_GSS;
        case 11: return SessionResult::MO_QUEUE_FULL;
        case 12: return SessionResult::TOO_MANY_SEGMENTS;
        case 13: return SessionResult::SESSION_INCOMPLETE_GSS;
        case 14: return SessionResult::INVALID_SEGMENT_SIZE;
        case 15: return SessionResult::ACCESS_DENIED;
        case 16: return SessionResult::ISU_LOCKED;
        case 17: return SessionResult::GATEWAY_NOT_RESPONDING;
        case 18: return SessionResult::CONNECTION_LOST;
        case 19: return SessionResult::LINK_FAILURE;
        case 32: return SessionResult::NO_NETWORK;
        case 33: return SessionResult::ANTENNA_FAULT;
        case 34: return SessionResult::RADIO_DISABLED;
        case 35: return SessionResult::ISU_BUSY;
        case 36: return SessionResult::TRY_LATER_REGISTRATION;
        case 37: return SessionResult::SBD_SERVICE_DISABLED;
        case 38: return SessionResult::TRY_LATER_TRAFFIC;
        case 64: return SessionResult::BAND_VIOLATION;
        case 65: return SessionResult::PLL_LOCK_FAILURE;
        default: return SessionResult::UNKNOWN_ERROR;
    }
}

bool message_available()
{
    if (!_initialized)
    {
        _last_error = DriverError::NOT_INITIALIZED;
        return false;
    }

    uint64_t t0 = now_ms();
    port_send("AT+SBDSX\r");

    char     line[128];
    uint64_t deadline = now_ms() + _at_timeout;
    int      mo_flag = 0, momsn = 0, mt_flag = 0, mtmsn = 0;
    bool     parsed  = false;

    while (now_ms() < deadline)
    {
        if (!port_read_line(line, sizeof(line), 100)) continue;
        if (sscanf(line, "+SBDSX: %d, %d, %d, %d",
                   &mo_flag, &momsn, &mt_flag, &mtmsn) == 4 ||
            sscanf(line, "+SBDSX:%d,%d,%d,%d",
                   &mo_flag, &momsn, &mt_flag, &mtmsn) == 4)
        {
            parsed = true;
            break;
        }
        if (strstr(line, "ERROR")) break;
    }

    if (!parsed)
    {
        printf("[SBDSX] TIMEOUT (%llums)\n",
               (unsigned long long)(now_ms() - t0));
        _last_error = DriverError::UART_TIMEOUT;
        return false;
    }

    printf("[SBDSX] mt_flag=%d (%llums)\n",
           mt_flag, (unsigned long long)(now_ms() - t0));

    _mt_pending = (mt_flag != 0);
    _last_error = DriverError::OK;
    return _mt_pending;
}

bool read_message(uint8_t* buffer, uint16_t max_length, uint16_t* received)
{
    if (!_initialized)
    {
        _last_error = DriverError::NOT_INITIALIZED;
        return false;
    }

    uint64_t t0 = now_ms();
    port_send("AT+SBDRB\r");

    uint8_t b0 = 0, b1 = 0;
    if (!uart_read_byte(&b0, _at_timeout) ||
        !uart_read_byte(&b1, _at_timeout))
    {
        printf("[SBDRB] TIMEOUT reading length (%llums)\n",
               (unsigned long long)(now_ms() - t0));
        _last_error = DriverError::UART_TIMEOUT;
        return false;
    }

    uint16_t len = (uint16_t)(((uint16_t)b0 << 8) | (uint16_t)b1);
    if (len == 0)         { _last_error = DriverError::NO_MESSAGE_WAITING; return false; }
    if (len > max_length) { _last_error = DriverError::READ_FAILED;        return false; }

    uint16_t computed = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t bi = 0;
        if (!uart_read_byte(&bi, _at_timeout))
        {
            _last_error = DriverError::UART_TIMEOUT;
            return false;
        }
        buffer[i] = bi;
        computed  = (uint16_t)(computed + bi);
    }

    uint8_t cs0 = 0, cs1 = 0;
    if (!uart_read_byte(&cs0, _at_timeout) ||
        !uart_read_byte(&cs1, _at_timeout))
    {
        _last_error = DriverError::UART_TIMEOUT;
        return false;
    }

    uint16_t rx_checksum = (uint16_t)(((uint16_t)cs0 << 8) | (uint16_t)cs1);
    if (rx_checksum != computed)
    {
        _last_error = DriverError::READ_FAILED;
        return false;
    }

    printf("[SBDRB] read %u bytes OK (%llums)\n",
           (unsigned)len, (unsigned long long)(now_ms() - t0));

    *received   = len;
    _mt_pending = false;
    _last_error = DriverError::OK;
    return true;
}

bool clear_buffers()
{
    return send_at_retry("AT+SBDD2");
}

bool sleep()
{
    if (!_initialized)
    {
        _last_error = DriverError::NOT_INITIALIZED;
        return false;
    }
    set_sleep_pin(false);
    printf("[sleep] modem sleep\n");
    return true;
}

bool wake()
{
    if (!_initialized)
    {
        _last_error = DriverError::NOT_INITIALIZED;
        return false;
    }
    set_sleep_pin(true);
    printf("[wake] waiting 1500ms...\n");
    sleep_ms(1500);
    printf("[wake] done\n");
    return true;
}


bool abort_session()
{
    port_send("+++");
    sleep_ms(100);

    if (send_at("ATH"))
    {
        _session_active = false;
        return true;
    }

    _session_active = false;
    return false;
}


bool hardware_reset()
{
    return software_reset();
}

bool software_reset()
{
    return send_at_retry("ATZ");
}


DriverError last_error() { return _last_error; }

const char* error_string(DriverError err)
{
    switch (err)
    {
        case DriverError::OK:                   return "OK";
        case DriverError::NOT_INITIALIZED:      return "Not initialized";
        case DriverError::UART_TIMEOUT:         return "UART timeout";
        case DriverError::MODEM_NOT_RESPONDING: return "Modem not responding";
        case DriverError::PAYLOAD_TOO_LARGE:    return "Payload too large";
        case DriverError::BUFFER_WRITE_FAILED:  return "Buffer write failed";
        case DriverError::SESSION_IN_PROGRESS:  return "Session in progress";
        case DriverError::SESSION_FAILED:       return "Session failed";
        case DriverError::NO_MESSAGE_WAITING:   return "No message waiting";
        case DriverError::READ_FAILED:          return "Read failed";
        case DriverError::SLEEP_FAILED:         return "Sleep failed";
        case DriverError::RESET_FAILED:         return "Reset failed";
        default:                                return "Unknown";
    }
}

} // namespace iridium_driver