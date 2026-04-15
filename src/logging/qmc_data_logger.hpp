#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <stdint.h>

/* ============================================================
  MagMovingAverage
   Simple ring-buffer moving average filter.
   One instance per sensor axis — X, Y, Z filtered independently
   so noise on one axis does not corrupt the others.

   Window size is set at construction time.
   At 50 Hz ODR, window=8 → ~160 ms of smoothing.

   Buffer capacity is MAX_WINDOW (32 samples).
   _window controls how many slots are actually used —
   slots beyond _window are never written or read.

   reset() must be called whenever calibration offsets change
   so stale pre-calibration samples are flushed from the buffer.
   ============================================================ */
class MagMovingAverage
{
public:
    // Maximum allowed window size — buffer is sized to this.
    // Passing window > MAX_WINDOW clamps to MAX_WINDOW safely.
    static constexpr uint8_t MAX_WINDOW = 32;

    MagMovingAverage(uint8_t window)
        : _window(window < MAX_WINDOW ? window : MAX_WINDOW),
          _sum(0.0f), _count(0), _index(0)
    {
        // Only clear slots that will actually be used.
        // Slots beyond _window are never touched by update() or reset().
        for (int i = 0; i < _window; i++) _buf[i] = 0.0f;
    }

    float update(float val)
    {
        _sum -= _buf[_index];
        _buf[_index] = val;
        _sum += val;
        _index = (_index + 1) % _window;   // wraps at _window, not MAX_WINDOW
        if (_count < _window) _count++;     // ramps up to _window on first fill
        return _sum / _count;
    }

    void reset()
    {
        _sum = 0.0f; _count = 0; _index = 0;
        // Only clear slots that were actually used — same as constructor
        for (int i = 0; i < _window; i++) _buf[i] = 0.0f;
    }

private:
    uint8_t _window;            // active window size (≤ MAX_WINDOW)
    float   _buf[MAX_WINDOW];   // ring buffer — only [0.._window-1] used
    float   _sum;               // running sum of active window
    uint8_t _count;             // ramps 0 → _window on first fill
    uint8_t _index;             // write head — wraps at _window
};

/* ============================================================
   MagSample
   Final output struct from the MagAcquisition pipeline.
   All fields are fully processed:
     - Hard-iron + soft-iron calibration applied
     - Moving-average filter applied
     - Raw counts converted to Gauss
     - Compass heading computed from filtered XY

   Moved here (out of mag_acquisition.hpp) so MagDataLogger
   can reference it without creating a circular include:
     data_logger.hpp → mag_acquisition.hpp → data_logger.hpp ✗
     data_logger.hpp (self-contained)       → mag_acquisition.hpp ✓
   ============================================================ */
struct MagSample
{
    int16_t  raw_x;         // latest raw X counts from sensor
    int16_t  raw_y;         // latest raw Y counts from sensor
    int16_t  raw_z;         // latest raw Z counts from sensor
    float    x_gauss;       // calibrated + filtered X field (Gauss)
    float    y_gauss;       // calibrated + filtered Y field (Gauss)
    float    z_gauss;       // calibrated + filtered Z field (Gauss)
    float    heading_deg;   // 0–360° compass heading (XY plane)
    float    temperature;   // sensor die temperature °C (offset corrected)
    bool     overflow;      // true → field exceeded ±8G range, data unreliable
    bool     data_skipped;  // true → MCU missed a sample from the sensor
    uint64_t timestamp_ms;  // milliseconds since boot (to_ms_since_boot)
};

/* ============================================================
   MagLoggerMode
   Three-way switch controlling what stage of the pipeline
   gets printed. Use this to debug calibration step by step.

   PRE_CALIB  (2) — raw int16 counts straight from sensor.
                    Use this to collect min/max for hard-iron
                    calibration. Rotate sensor in figure-8,
                    watch min/max stabilize, then compute offsets.
                    Header:  raw_x, raw_y, raw_z

   POST_CALIB (1) — float counts after hard-iron subtraction
                    but before Gauss conversion or filtering.
                    + temperature for OBC monitoring.
                    Use this to verify calibration worked —
                    values should be centered around 0.
                    Header:  cal_x, cal_y, cal_z, temperature

   CONVERTED  (0) — full pipeline output. Calibrated + filtered
                    + Gauss + heading + temperature.
                    Header:  x_gauss, y_gauss, z_gauss,
                             heading_deg, temperature,
                             overflow, skipped, timestamp_ms

   Correct debug workflow:
     Step 1 → LOGGER_MODE 2 (PRE_CALIB)   collect raw min/max
     Step 2 → compute hard-iron offsets = (max+min)/2 per axis
     Step 3 → LOGGER_MODE 1 (POST_CALIB)  verify centered at 0
     Step 4 → LOGGER_MODE 0 (CONVERTED)   check heading is correct
   ============================================================ */
enum class MagLoggerMode : uint8_t
{
    CONVERTED  = 0,   // full pipeline — Gauss + heading + temperature
    POST_CALIB = 1,   // after hard-iron subtraction, before Gauss + temperature
    PRE_CALIB  = 2    // raw int16 counts — no processing at all
};

/* ============================================================
   MagDataLogger
   Single output boundary for all magnetometer serial data.
   Nothing else in the mag codebase should printf directly.

   Three print points matching the three pipeline stages:
    - logRaw()        → PRE_CALIB  — raw int16 counts
    - logCalibrated() → POST_CALIB — post hard-iron counts + temperature
    - log()           → CONVERTED  — full MagSample output

   printHeader() always matches the active mode automatically.
   ============================================================ */
class MagDataLogger
{
public:
    explicit MagDataLogger(MagLoggerMode mode);

    void          setMode(MagLoggerMode mode);
    MagLoggerMode getMode() const;

    /**
     * @brief Print CSV header matching the active mode.
     *        Call once before logging starts.
     *
     * PRE_CALIB  header:  raw_x,raw_y,raw_z
     * POST_CALIB header:  cal_x,cal_y,cal_z,temperature
     * CONVERTED  header:  x_gauss,y_gauss,z_gauss,heading_deg,
     *                     temperature,overflow,skipped,timestamp_ms
     */
    void printHeader() const;

    /**
     * @brief Log one fully-processed MagSample — CONVERTED mode.
     *        Prints Gauss + heading + temperature + status flags.
     *        Should not be called in PRE_CALIB or POST_CALIB mode
     *        as the pipeline exits early before reaching this point.
     */
    void log(const MagSample& sample) const;

    /**
     * @brief Log raw int16 counts — PRE_CALIB mode.
     *        Bypasses MagSample and the entire pipeline.
     *        Use during calibration to collect min/max per axis.
     *        Also used in run_calibration() in main.cpp.
     */
    void logRaw(int16_t x, int16_t y, int16_t z) const;

    /**
     * @brief Log post-hard-iron float counts + temperature — POST_CALIB mode.
     *        Called after hard-iron subtraction, before Gauss conversion.
     *        cal_x/y/z should sit near 0 if hard-iron calibration is correct.
     *        temperature included for OBC thermal monitoring at this stage.
     *
     *        If cal values are still offset → adjust hard-iron offsets.
     *        If elliptical when plotted → soft-iron correction needed.
     */
    void logCalibrated(float cx, float cy, float cz, float temp) const;

private:
    MagLoggerMode _mode;
};

/* ============================================================
   MagCalibData
   Hard-iron + soft-iron calibration stored in flash.

   magic       — 0xAB1234CD proves sector was written by us.
                 Blank/erased flash reads 0xFFFFFFFF — fails check.
   hard_x/y/z  — hard-iron offsets in raw ADC counts.
                 Subtracted from every reading before conversion.
   soft_iron   — 3x3 soft-iron matrix, row-major (9 floats).
                 Identity matrix = no soft-iron correction (default).
   crc         — XOR checksum over all offset + matrix data.
                 Catches flash bit corruption between writes.

   Layout in flash (separate sector from IMU calib):
     [0x1FE000  ← MAG_CALIB_FLASH_OFFSET  4KB sector for mag]
     [0x1FF000  ← IMU calib sector — untouched]
     [0x200000  ← end of 2MB flash]
   ============================================================ */
struct MagCalibData
{
    uint32_t magic;           // must equal MAG_CALIB_MAGIC to be valid
    int16_t  hard_x;          // hard-iron X offset (ADC counts)
    int16_t  hard_y;          // hard-iron Y offset (ADC counts)
    int16_t  hard_z;          // hard-iron Z offset (ADC counts)
    float    soft_iron[9];    // soft-iron matrix row-major, identity = no correction
    uint16_t crc;             // XOR checksum — catches flash bit corruption
};

/* ============================================================
   MagCalibStore
   Reads and writes MagCalibData to a dedicated 4KB flash sector
   one sector below the IMU calib sector so the two never collide.

   Flash layout (2MB chip):
     [0x000000 ... program code ...]
     [... empty flash ...]
     [0x1FE000  MAG calib  — 4KB]   ← this class
     [0x1FF000  IMU calib  — 4KB]   ← existing CalibStore
     [0x200000  end]

   Same RP2040 flash rules as IMU CalibStore:
    - Erase full 4KB sector before every write.
    - Write runs from RAM with interrupts disabled (SDK handles it).
    - ~100,000 write cycles — only written on calibration, safe.

   Usage:
     MagCalibData d;
     if (MagCalibStore::load(d)) {
         mag.setHardIronOffsets(d.hard_x, d.hard_y, d.hard_z);
         mag.setSoftIronMatrix(d.soft_iron);
     } else {
         run_mag_calibration();
         MagCalibStore::save(hx, hy, hz, soft_matrix);
     }
   ============================================================ */
class MagCalibStore
{
public:
    static constexpr uint32_t MAG_CALIB_MAGIC = 0xAB1234CD;

    // One sector below the IMU calib sector.
    // 0x200000 = 2MB flash end, 0x1000 = 4KB per sector.
    //   IMU sector: 0x200000 - 0x1000 = 0x1FF000
    //   MAG sector: 0x200000 - 0x2000 = 0x1FE000
    static constexpr uint32_t MAG_CALIB_FLASH_OFFSET = 0x200000 - 0x2000;

    /**
     * @brief Load calibration from flash.
     *        Validates magic and CRC — returns false if either fails,
     *        leaving data unchanged so caller can trigger recalibration.
     * @param data  Output — filled on success.
     * @return true if valid calibration was found.
     */
    static bool load(MagCalibData& data);

    /**
     * @brief Save hard-iron offsets + soft-iron matrix to flash.
     *        Fills magic and CRC automatically.
     *        Erases the sector then writes the full MagCalibData struct.
     * @param hard_x/y/z   Hard-iron offsets in ADC counts.
     * @param soft_iron     9-element row-major soft-iron matrix.
     */
    static void save(int16_t      hard_x,
                     int16_t      hard_y,
                     int16_t      hard_z,
                     const float  soft_iron[9]);

    /**
     * @brief Erase the mag calibration sector.
     *        Forces recalibration on next boot.
     *        Does NOT touch the IMU calib sector.
     */
    static void erase();

private:
    /**
     * @brief XOR checksum over hard-iron offsets + soft-iron matrix bytes.
     *        Same algorithm as IMU CalibStore for consistency.
     */
    static uint16_t computeCRC(int16_t     hard_x,
                                int16_t     hard_y,
                                int16_t     hard_z,
                                const float soft_iron[9]);
};

#endif