#ifndef LSM6DSV80X_DATA_LOGGER_HPP
#define LSM6DSV80X_DATA_LOGGER_HPP

#include <stdint.h>
#include "ls_conversion.hpp"   // LSM6DSV80X_Data, RawGY, RawHG

/* ============================================================
   LSM6DSV80X_LoggerMode
   Selects output format for LSM6DSV80X_DataLogger.

   RAW       = int16 ADC counts
                 (e.g.  312, -44, 1024, 88, -201, 9983, 128)
   CONVERTED = physical units
                 GY   → dps      (e.g.  10.9200, -1.5400,  0.0350)
                 HG   → g        (e.g.   0.0000, -0.0010,  1.0000)
                 TEMP → celsius  (e.g.  25.500)
   ============================================================ */

enum class LSM6DSV80X_LoggerMode : uint8_t
{
    RAW       = 0,
    CONVERTED = 1
};

/* ============================================================
   LSM6DSV80X_DataLogger
   Single output boundary for all LSM6DSV80X serial data.
   Nothing else in the codebase should printf sensor data
   directly — route everything through here.

   Channels logged:
     - Gyroscope     (GY)   — dps      when CONVERTED
     - High-G accel  (HG)   — g        when CONVERTED
     - Temperature   (TEMP) — celsius  when CONVERTED

   CSV column order:
     RAW:        hg_x, hg_y, hg_z, gy_x, gy_y, gy_z, temp_raw
     CONVERTED:  hg_x[g],   hg_y[g],   hg_z[g],
                 gy_x[dps], gy_y[dps], gy_z[dps],
                 temp[C]
   ============================================================ */

class LSM6DSV80X_DataLogger
{
public:

    explicit LSM6DSV80X_DataLogger(LSM6DSV80X_LoggerMode mode = LSM6DSV80X_LoggerMode::CONVERTED);

    void                  setMode(LSM6DSV80X_LoggerMode mode);
    LSM6DSV80X_LoggerMode getMode() const;

    /**
     * @brief Print CSV header matching the active mode.
     *        Call once at startup before logging begins.
     */
    void printHeader() const;

    /**
     * @brief Log one sample to serial output.
     *
     * In RAW mode:       raw counts printed, converted ignored.
     * In CONVERTED mode: physical-unit values printed, raw ignored.
     *
     * Temperature conversion: temp_celsius = (raw_temp / 256.0f) + 25.0f
     * The offset stored in CalibData is subtracted from raw_temp before
     * conversion so the reading is corrected at the source.
     *
     * @param raw_gy    Raw gyroscope ADC counts.
     * @param raw_hg    Raw high-G accelerometer ADC counts.
     * @param raw_temp  Raw temperature ADC count (offset already applied).
     * @param conv_gy   Converted gyroscope data  (dps).
     * @param conv_hg   Converted high-G data     (g).
     */
    void log(const LSM6DSV80X_RawGY& raw_gy,
             const LSM6DSV80X_RawHG& raw_hg,
             int16_t                 raw_temp,
             const LSM6DSV80X_Data&  conv_gy,
             const LSM6DSV80X_Data&  conv_hg) const;

private:

    LSM6DSV80X_LoggerMode _mode;
};

/* ============================================================
   LSM6DSV80X_CalibData
   Calibration offsets for gyroscope, high-G accelerometer,
   and temperature sensor — all stored in raw ADC counts.

   magic      — 0x16D5CA1B proves flash was written by this
                driver. Blank flash reads 0xFFFFFFFF — any
                value other than the magic = treat as invalid.

   gy_off_*   — gyroscope offsets in raw ADC counts.
   hg_off_*   — high-G accelerometer offsets in raw ADC counts.
   temp_off   — temperature offset in raw ADC counts.
                Applied as: corrected_raw = raw - temp_off
                So that readings are relative to the calibration
                baseline rather than the absolute sensor zero.

   crc        — XOR checksum across all seven offset fields.
                If stored crc != computed crc → treat as
                corrupt and force recalibration.
   ============================================================ */

struct LSM6DSV80X_CalibData
{
    uint32_t magic;

    int16_t  gy_off_x;
    int16_t  gy_off_y;
    int16_t  gy_off_z;

    int16_t  hg_off_x;
    int16_t  hg_off_y;
    int16_t  hg_off_z;

    int16_t  temp_off;  // raw ADC count offset — subtract from raw before converting

    uint16_t crc;       // XOR checksum of all seven offsets above
};

/* ============================================================
   LSM6DSV80X_CalibStore
   Reads and writes LSM6DSV80X_CalibData to the last 4KB
   sector of RP2040 internal flash.

   Flash layout:
     [0x000000  ... program code ...]
     [... empty ...]
     [0x1FF000  ← CALIB_FLASH_OFFSET  4KB sector for calib]

   RP2040 flash rules:
     - Cannot execute from flash while erasing/writing.
       Pico SDK runs those routines from RAM with interrupts
       disabled automatically.
     - Must erase full 4KB sector before any write.
     - Write endurance: ~100,000 cycles.
       Only written on calibration — well within limits.

   Usage:
     LSM6DSV80X_CalibData d;
     if (LSM6DSV80X_CalibStore::load(d))
     {
         acquisition.setGYOffsets(d.gy_off_x, d.gy_off_y, d.gy_off_z);
         acquisition.setHGOffsets(d.hg_off_x, d.hg_off_y, d.hg_off_z);
         acquisition.setTempOffset(d.temp_off);
     }
     else
     {
         // run calibration, then:
         LSM6DSV80X_CalibStore::save(gy_x, gy_y, gy_z,
                                     hg_x, hg_y, hg_z,
                                     temp_off);
     }
   ============================================================ */

class LSM6DSV80X_CalibStore
{
public:

    static constexpr uint32_t CALIB_MAGIC        = 0x16D5CA1B;
    static constexpr uint32_t CALIB_FLASH_OFFSET = 0x200000 - 0x1000; // last 4KB of 2MB flash

    /**
     * @brief Attempt to load calibration from flash.
     *
     * Checks magic number and CRC. If either is wrong,
     * returns false and leaves data unchanged.
     *
     * @param data  Output — filled on success.
     * @return true if valid calibration data was found.
     */
    static bool load(LSM6DSV80X_CalibData& data);

    /**
     * @brief Save calibration offsets to flash.
     *
     * Fills magic and computes CRC automatically.
     * Erases the sector then writes the struct.
     *
     * @param gy_x/y/z   Gyroscope offsets in ADC counts.
     * @param hg_x/y/z   High-G accelerometer offsets in ADC counts.
     * @param temp_off   Temperature offset in ADC counts.
     */
    static void save(int16_t gy_x,   int16_t gy_y,   int16_t gy_z,
                     int16_t hg_x,   int16_t hg_y,   int16_t hg_z,
                     int16_t temp_off);

    /**
     * @brief Erase the calibration sector.
     *        Forces recalibration on next boot.
     */
    static void erase();

    /**
     * @brief Quick validity check without loading the full struct.
     * @return true if magic and CRC are both valid.
     */
    static bool isValid();

private:

    /**
     * @brief XOR checksum across all seven int16 offset fields.
     *
     * Example:
     *   gy_x=0x0004, gy_y=0x00E3, gy_z=0xFFD4,
     *   hg_x=0x0012, hg_y=0xFF01, hg_z=0x0033,
     *   temp_off=0x0080
     *   crc = XOR of all seven
     */
    static uint16_t computeCRC(int16_t gy_x,   int16_t gy_y,   int16_t gy_z,
                                int16_t hg_x,   int16_t hg_y,   int16_t hg_z,
                                int16_t temp_off);
};

#endif // LSM6DSV80X_DATA_LOGGER_HPP