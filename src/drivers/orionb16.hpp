#ifndef ORION_B16_HPP
#define ORION_B16_HPP
/*

UART RX pin
    │
    ▼
update() called in loop
    │  (one char at a time)
    ▼
encode(char c)
    │  (buffers until '\n')
    ▼
parseSentence()
    │  uses getField(), parseCoordinate(), toUnixEpoch()
    ▼
_current_data updated
    │
    ▼
getData(out) → your application

*/
#include <stdint.h>
#include "hardware/uart.h"
#include "pico/stdlib.h"
// are there any constraints like delay time for boot up or anything to be domne particularly for this sensor?
class OrionB16 {
public:
    static constexpr uint8_t MAX_NMEA_LEN = 82;
    static constexpr uint32_t DEFAULT_BAUD = 9600; // so why used 9600 baud rate only?
// unix_timestamp, lat, long, altitude onlu added into the struct
    // Updated Flight Data Struct
    struct GNSSData {
        uint32_t unix_timestamp; // Seconds since Jan 1, 1970
        int32_t  latitude;       // Decimal degrees scaled by 10^7
        int32_t  longitude;      // Decimal degrees scaled by 10^7
        int32_t  altitude;       // Altitude in cm
        int32_t  velocity;       // Speed over ground in cm/s
        uint8_t  satellites;     // Number of connected satellites
        bool     valid;          // True if we have a 3D Fix - checking if there are atleast 4 satellites .. not to trust any fields if this is wrong
    };

    OrionB16(uart_inst_t* uart_inst);

    bool init(uint tx_pin, uint rx_pin, uint baud_rate = DEFAULT_BAUD);
    void update();  // tells if there are any available charecters in uart
    bool getData(GNSSData& out_data); // call to get the recent parsed data

private:
    uart_inst_t* _uart;
    char         _buffer[MAX_NMEA_LEN]; // char buffer to accumulate data uart
    uint8_t      _buffer_idx;
    GNSSData     _current_data;

    // Cache variables to assemble the UNIX timestamp across different NMEA sentences until both are available
    uint32_t     _last_time; 
    uint32_t     _last_date;

    void     encode(char c); //  Appends to _buffer; when it sees \n
    void     parseSentence(); // identifies nmea fields and extracts into particular type
    char* getField(char* sentence, uint8_t field_num); // NMEA sentences are comma-delimited. This is a utility to return a pointer to the N-th field in the raw sentence string
    int32_t  parseCoordinate(char* nmea_str, char* dir_str); // converts nmea raw+ direction into scaled integer format
    uint32_t toUnixEpoch(uint32_t date_ddmmyy, uint32_t time_hhmmss); // converts date and time into standard unix time  format
};

#endif