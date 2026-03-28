#ifndef KX_DATA_LOGGER_HPP
#define KX_DATA_LOGGER_HPP

#include <stdint.h>
#include "kx134.hpp"
#include "imu_conversion.hpp"

/* ============================================================
   LoggerMode
   Selects output format for DataLogger.
   RAW      = int16 ADC counts  (e.g.  4, 227, -44)
   CONVERTED = float in g       (e.g.  0.000, -0.001, 1.000)
   ============================================================ */

enum class LoggerMode : uint8_t
{
    RAW       = 0,
    CONVERTED = 1
};

/* ============================================================
   DataLogger
   Single output boundary of the system.
   All serial data output goes through here — nothing else
   in the codebase should printf data directly.

   Changes from original:
    - timestamp_us parameter removed (was unused)
    - printHeader() added — header now matches the active mode
      automatically instead of being hardcoded in main.cpp
   ============================================================ */

class DataLogger
{
public:

    explicit DataLogger(LoggerMode mode);

    void       setMode(LoggerMode mode);
    LoggerMode getMode() const;

    /**
     * @brief Print CSV header matching the active mode.
     *        Call once in system_init() before logging starts.
     */
    void printHeader() const;

    /**
     * @brief Log one sample to serial output.
     *
     * timestamp_us removed — was accepted but never printed.
     * If timestamp logging is re-enabled later, add it back
     * here and update the call site in imu_acquisition.cpp.
     */
    void log(const KX134_Raw&  raw,
             const IMU_Data&   converted) const;

private:

    LoggerMode _mode;
};


/* ============================================================
   CalibData
   Calibration offsets stored in RP2040 internal flash.

   magic   — 0xCAFE1234 proves flash was written by us.
             On a blank chip flash reads as 0xFFFFFFFF —
             any value other than the magic = treat as invalid.
   off_x/y/z — calibration offsets in raw ADC counts.
   crc     — simple XOR checksum catches bit corruption.
             If stored crc != computed crc → treat as corrupt,
             force recalibration.
   ============================================================ */

struct CalibData
{
    uint32_t magic;       // must equal CALIB_MAGIC to be valid
    int16_t  off_x;
    int16_t  off_y;
    int16_t  off_z;
    uint16_t crc;         // XOR checksum of off_x, off_y, off_z
};

/* ============================================================
   CalibStore
   Reads and writes CalibData to the last 4KB sector of
   RP2040 internal flash.

   Flash layout:
     [0x000000 ... program code ...]
     [... empty ...]
     [0x1FF000  ← CALIB_FLASH_OFFSET  4KB sector for calib]

   RP2040 flash rules:
    - Cannot write flash while executing from flash.
      The Pico SDK handles this by running erase/write
      routines from RAM with interrupts disabled.
    - Must erase entire 4KB sector before writing.
    - Write lifetime: ~100,000 cycles.
      We only write on calibration — safe.

   Usage:
     CalibData d;
     if (CalibStore::load(d))   // true = valid data found
         use d.off_x/y/z
     else
         run calibration, then CalibStore::save(d)
   ============================================================ */

class CalibStore
{
public:

    static constexpr uint32_t CALIB_MAGIC        = 0xCAFE1234;

    // Last 4KB sector of 2MB flash.
    // 0x200000 = total flash size, 0x1000 = 4KB sector size.
    static constexpr uint32_t CALIB_FLASH_OFFSET = 0x200000 - 0x1000;

    /**
     * @brief Attempt to load calibration from flash.
     *
     * Checks magic number and CRC. If either is wrong,
     * returns false and leaves data unchanged.
     *
     * @param data  Output — filled on success.
     * @return true if valid calibration data was found.
     */
    static bool load(CalibData& data);

    /**
     * @brief Save calibration offsets to flash.
     *
     * Fills magic and computes CRC automatically.
     * Erases the sector then writes the struct.
     *
     * @param off_x  X axis offset in ADC counts.
     * @param off_y  Y axis offset in ADC counts.
     * @param off_z  Z axis offset in ADC counts.
     */
    static void save(int16_t off_x,
                     int16_t off_y,
                     int16_t off_z);

    /**
     * @brief Erase the calibration sector.
     *        Forces recalibration on next boot.
     */
    static void erase();

private:

    /**
     * @brief Compute XOR checksum of the three offsets.
     *        Simple but sufficient for flash corruption detection.
     */
    static uint16_t computeCRC(int16_t off_x,
                                int16_t off_y,
                                int16_t off_z);
};

#endif