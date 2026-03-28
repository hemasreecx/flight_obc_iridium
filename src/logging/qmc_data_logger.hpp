#ifndef QMC_DATA_LOGGER_HPP
#define QMC_DATA_LOGGER_HPP

#include <stdint.h>

/* ============================================================
   MovingAverage
   Simple ring-buffer moving average filter.
   One instance per sensor axis — X, Y, Z filtered independently
   so noise on one axis does not corrupt the others.

   Window size is set at construction time.
   At 50 Hz ODR, window=8 → ~160 ms of smoothing.

   reset() must be called whenever calibration offsets change
   so stale pre-calibration samples are flushed from the buffer.
   ============================================================ */
class MagMovingAverage
{
public:
    MagMovingAverage(uint8_t window)
        : _window(window), _sum(0.0f), _count(0), _index(0)
    {
        for (int i = 0; i < 32; i++) _buf[i] = 0.0f;
    }

    float update(float val)
    {
        _sum -= _buf[_index];
        _buf[_index] = val;
        _sum += val;
        _index = (_index + 1) % _window;
        if (_count < _window) _count++;
        return _sum / _count;
    }

    void reset()
    {
        _sum = 0.0f; _count = 0; _index = 0;
        for (int i = 0; i < 32; i++) _buf[i] = 0.0f;
    }

private:
    uint8_t _window;
    float   _buf[32];   // max window size = 32 samples
    float   _sum;
    uint8_t _count;     // ramps up from 0 to _window on first fill
    uint8_t _index;     // write head — wraps at _window
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
    float    x_gauss;       // calibrated + filtered X field (Gauss)
    float    y_gauss;       // calibrated + filtered Y field (Gauss)
    float    z_gauss;       // calibrated + filtered Z field (Gauss)
    float    heading_deg;   // 0–360° compass heading (XY plane)
    float    temperature;   // sensor die temperature °C
    bool     overflow;      // true → field exceeded ±8G range, data unreliable
    bool     data_skipped;  // true → MCU missed a sample from the sensor
    uint64_t timestamp_ms;  // milliseconds since boot (to_ms_since_boot)
};

/* ============================================================
   MagLoggerMode
   Selects output format for MagDataLogger.

   RAW       = int16 ADC counts before calibration
               e.g.  raw_x=1240, raw_y=-530, raw_z=890
   CONVERTED = calibrated Gauss + heading + temperature
               e.g.  x=0.413G, y=-0.177G, z=0.297G, hdg=156.8°
   ============================================================ */
enum class MagLoggerMode : uint8_t
{
    RAW       = 0,
    CONVERTED = 1
};

/* ============================================================
   MagDataLogger
   Single output boundary for all magnetometer serial data.
   Nothing else in the mag codebase should printf directly —
   all data exits the system through log() or logRaw() here.

   Mirrors the IMU DataLogger design:
    - Same RAW/CONVERTED mode switch
    - printHeader() matches active mode automatically
    - log() is the only way processed data leaves the system
    - logRaw() bypasses the pipeline for calibration routines
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
     * RAW header:
     *   raw_x,raw_y,raw_z,overflow,skipped,timestamp_ms
     *
     * CONVERTED header:
     *   x_gauss,y_gauss,z_gauss,heading_deg,temperature,overflow,skipped,timestamp_ms
     */
    void printHeader() const;

    /**
     * @brief Log one fully-processed MagSample.
     *        In RAW mode:       prints Gauss values scaled back to counts.
     *        In CONVERTED mode: prints Gauss + heading + temperature.
     *
     *        Note: RAW mode re-scales from Gauss using SCALE=3000 LSB/G
     *        because MagSample stores calibrated floats, not raw int16.
     *        For true pre-calibration counts use logRaw() directly.
     */
    void log(const MagSample& sample) const;

    /**
     * @brief Log raw counts directly — bypasses MagSample entirely.
     *        Used during calibration routines before the acquisition
     *        pipeline is active, so raw sensor output can be verified.
     */
    void logRaw(int16_t x, int16_t y, int16_t z) const;

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
     *
     * @param data  Output — filled on success.
     * @return true if valid calibration was found.
     */
    static bool load(MagCalibData& data);

    /**
     * @brief Save hard-iron offsets + soft-iron matrix to flash.
     *        Fills magic and CRC automatically.
     *        Erases the sector then writes the full MagCalibData struct.
     *
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
     *        Covers the full bit pattern of every float via uint16 pairs.
     */
    static uint16_t computeCRC(int16_t     hard_x,
                                int16_t     hard_y,
                                int16_t     hard_z,
                                const float soft_iron[9]);
};

#endif