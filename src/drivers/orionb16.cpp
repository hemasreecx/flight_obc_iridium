#include "orionb16.hpp"
#include <string.h>
#include <stdlib.h>

OrionB16::OrionB16(uart_inst_t* uart_inst)
    : _uart(uart_inst),
      _buffer_idx(0),
      _last_time(0),
      _last_date(0)
{
    memset(_buffer, 0, sizeof(_buffer)); //clears all the charecter buffers to null
    _current_data.valid = false;
    _current_data.satellites = 0;
}

bool OrionB16::init(uint tx_pin, uint rx_pin, uint baud_rate) {
    if (_uart == nullptr) return false;

    uart_init(_uart, baud_rate);
    gpio_set_function(tx_pin, GPIO_FUNC_UART); // tells the pico to route the gpio mux to uart not to gpio
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    
    uart_set_hw_flow(_uart, false, false); // Disables RTS/CTS hardware flow control — the B16 doesn't use it
    uart_set_format(_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(_uart, false); // disable the hardware buffer
    
    return true;
}

void OrionB16::update() {
    if (_uart == nullptr) return;
    while (uart_is_readable(_uart)) {
        encode(uart_getc(_uart));
    }
}
// It drains all available bytes in a single update() call, 

bool OrionB16::getData(GNSSData& out_data) {
    out_data = _current_data;
    return out_data.valid;
}

void OrionB16::encode(char c) {
    if (c == '$') _buffer_idx = 0;
    
    if (_buffer_idx < MAX_NMEA_LEN - 1) {
        _buffer[_buffer_idx++] = c;
    }

    if (c == '\n') {
        _buffer[_buffer_idx] = '\0';
        parseSentence();
    }
}

char* OrionB16::getField(char* sentence, uint8_t field_num) {
    uint8_t current = 0;
    char* ptr = sentence;
    while (*ptr != '\0') {
        if (current == field_num) return ptr;
        if (*ptr == ',') current++;
        ptr++;
    }
    return nullptr;
}

int32_t OrionB16::parseCoordinate(char* nmea_str, char* dir_str) {
    if (!nmea_str || !dir_str || *nmea_str == '\0' || *dir_str == '\0') return 0;
    
    float raw = atof(nmea_str);
    int degrees = (int)(raw / 100.0f);
    float minutes = raw - (degrees * 100.0f);
    float decimal = degrees + (minutes / 60.0f);
    
    if (dir_str[0] == 'S' || dir_str[0] == 'W') decimal = -decimal;
    return (int32_t)(decimal * 1e7f); 
}

// Lightweight bare-metal UNIX epoch calculator (No heavy <time.h> reliance)
uint32_t OrionB16::toUnixEpoch(uint32_t date, uint32_t time) {
    if (date == 0 || time == 0) return 0;

    uint8_t d = date / 10000;
    uint8_t m = (date / 100) % 100;
    uint16_t y = (date % 100) + 2000; // GNSS sends 2-digit year (e.g. 26 = 2026)

    uint8_t hh = time / 10000;
    uint8_t mm = (time / 100) % 100;
    uint8_t ss = time % 100;

    const uint16_t days_in_month[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    
    uint32_t days = (y - 1970) * 365;
    days += (y - 1969) / 4; // Add leap days since 1970
    
    if ((y % 4 == 0) && (m <= 2)) days--; // Adjust if current year is leap but we are in Jan/Feb

    days += days_in_month[m - 1] + (d - 1);
    return (days * 86400) + (hh * 3600) + (mm * 60) + ss;
}

void OrionB16::parseSentence() {
    // This function identifies and handles two NMEA sentence types.  GGA — Position Fix Data
    // 1. Parse GGA for Location, Altitude, and Satellite Count
    if (strncmp(_buffer, "$GPGGA", 6) == 0 || strncmp(_buffer, "$GNGGA", 6) == 0) {
        char* fix_str = getField(_buffer, 6);
        uint8_t fix = fix_str ? atoi(fix_str) : 0;

        if (fix > 0) {
            _current_data.latitude = parseCoordinate(getField(_buffer, 2), getField(_buffer, 3));
            _current_data.longitude = parseCoordinate(getField(_buffer, 4), getField(_buffer, 5));

            char* alt_str = getField(_buffer, 9);
            if (alt_str) _current_data.altitude = (int32_t)(atof(alt_str) * 100.0f); // Convert M to CM

            char* sat_str = getField(_buffer, 7);
            if (sat_str) _current_data.satellites = (uint8_t)atoi(sat_str);

            char* time_str = getField(_buffer, 1);
            if (time_str) _last_time = (uint32_t)atol(time_str);

            _current_data.valid = true;
        } else {
            _current_data.valid = false;
        }
    } 
    // RMC — Recommended Minimum Sentence
    // 2. Parse RMC for Velocity and Date (to build the UNIX Timestamp)
    else if (strncmp(_buffer, "$GPRMC", 6) == 0 || strncmp(_buffer, "$GNRMC", 6) == 0) {
        char* status_str = getField(_buffer, 2);
        if (status_str && status_str[0] == 'A') { // 'A' means Active/Valid Data
            
            char* speed_str = getField(_buffer, 7);
            // GNSS outputs knots. 1 knot = 51.4444 cm/s
            if (speed_str) _current_data.velocity = (int32_t)(atof(speed_str) * 51.4444f); 

            char* date_str = getField(_buffer, 9);
            if (date_str) _last_date = (uint32_t)atol(date_str);

            char* time_str = getField(_buffer, 1);
            if (time_str) _last_time = (uint32_t)atol(time_str);

            // Assemble the UNIX Timestamp if we have both halves
            if (_last_time > 0 && _last_date > 0) {
                _current_data.unix_timestamp = toUnixEpoch(_last_date, _last_time);
            }
        }
    }
}

// GGA  AND RMC GONNA COME ONE AFGTER THE OTHER ->. NOP SINGLE SENTENCE CAN DO ANYTHING!