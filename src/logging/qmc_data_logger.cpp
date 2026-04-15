#include "../logging/data_logger.hpp"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include <stdio.h>
#include <string.h>

// ═════════════════════════════════════════════════════════════════════════════
// MagDataLogger
// ═════════════════════════════════════════════════════════════════════════════

MagDataLogger::MagDataLogger(MagLoggerMode mode)
    : _mode(mode)
{
}

void MagDataLogger::setMode(MagLoggerMode mode)
{
    _mode = mode;
}

MagLoggerMode MagDataLogger::getMode() const
{
    return _mode;
}

// ── printHeader() ─────────────────────────────────────────────────────────────

void MagDataLogger::printHeader() const
{
    if (_mode == MagLoggerMode::PRE_CALIB)
    {
        // Raw counts — no processing at all.
        // Use to collect min/max for hard-iron offset computation:
        //   offset = (max + min) / 2 per axis
        printf("raw_x,raw_y,raw_z\n");
    }
    else if (_mode == MagLoggerMode::POST_CALIB)
    {
        // Post hard-iron subtraction, pre-Gauss, pre-filter.
        // cal values should be centered around 0 after good calibration.
        // Temperature included for OBC thermal monitoring.
        printf("cal_x,cal_y,cal_z,temperature\n");
    }
    else
    {
        // Full pipeline — Gauss + heading + temperature + status
        printf("x_gauss,y_gauss,z_gauss,"
               "heading_deg,temperature,"
               "overflow,skipped,"
               "timestamp_ms\n");
    }
}

// ── log() — CONVERTED mode ────────────────────────────────────────────────────

void MagDataLogger::log(const MagSample& sample) const
{
    // Full pipeline output — Gauss values, heading, temperature, status flags.
    // Only reached in CONVERTED mode — PRE_CALIB and POST_CALIB exit early.
    printf("%.4f,%.4f,%.4f,%.2f,%.2f,%d,%d,%llu\n",
           sample.x_gauss,
           sample.y_gauss,
           sample.z_gauss,
           sample.heading_deg,
           sample.temperature,
           (int)sample.overflow,
           (int)sample.data_skipped,
           sample.timestamp_ms);
}

// ── logRaw() — PRE_CALIB mode ─────────────────────────────────────────────────

void MagDataLogger::logRaw(int16_t x, int16_t y, int16_t z) const
{
    // Raw int16 counts straight from sensor — no calibration, no conversion.
    // Use this to collect min/max values for hard-iron offset computation:
    //   offset_x = (max_x + min_x) / 2
    //   offset_y = (max_y + min_y) / 2
    //   offset_z = (max_z + min_z) / 2
    printf("%d,%d,%d\n", x, y, z);
}

// ── logCalibrated() — POST_CALIB mode ────────────────────────────────────────

void MagDataLogger::logCalibrated(float cx, float cy, float cz, float temp) const
{
    // Post hard-iron subtraction counts + OBC temperature.
    // cal values are still in raw count units but centered around 0.
    //
    // How to interpret:
    //   Good calibration  → cx, cy, cz all hover near 0 when stationary
    //   Still offset      → hard-iron values need adjustment
    //   Elliptical plot   → soft-iron correction needed (use Magneto tool)
    //
    // Temperature is included here so OBC thermal state can be monitored
    // even during the calibration verification phase.
    printf("%.2f,%.2f,%.2f,%.2f\n", cx, cy, cz, temp);
}

// ═════════════════════════════════════════════════════════════════════════════
// MagCalibStore
// ═════════════════════════════════════════════════════════════════════════════

bool MagCalibStore::load(MagCalibData& data)
{
    // Flash is memory-mapped on RP2040 — read directly via pointer.
    // XIP (execute-in-place) base address + our sector offset.
    const uint8_t* flash_ptr = reinterpret_cast<const uint8_t*>(
        0x10000000u + MAG_CALIB_FLASH_OFFSET);

    MagCalibData stored;
    memcpy(&stored, flash_ptr, sizeof(MagCalibData));

    // ── Validate magic ────────────────────────────────────────────────────────
    // Blank flash reads as 0xFFFFFFFF — any value other than our magic
    // means this sector was never written by us.
    if (stored.magic != MAG_CALIB_MAGIC)
        return false;

    // ── Validate CRC ──────────────────────────────────────────────────────────
    // Catches bit-level flash corruption between writes.
    uint16_t expected = computeCRC(stored.hard_x,
                                   stored.hard_y,
                                   stored.hard_z,
                                   stored.soft_iron);
    if (stored.crc != expected)
        return false;

    data = stored;
    return true;
}

void MagCalibStore::save(int16_t     hard_x,
                          int16_t     hard_y,
                          int16_t     hard_z,
                          const float soft_iron[9])
{
    MagCalibData data;
    data.magic  = MAG_CALIB_MAGIC;
    data.hard_x = hard_x;
    data.hard_y = hard_y;
    data.hard_z = hard_z;
    memcpy(data.soft_iron, soft_iron, sizeof(data.soft_iron));
    data.crc = computeCRC(hard_x, hard_y, hard_z, soft_iron);

    // Pad to full 4KB sector — flash_range_program() requires
    // buffer to be a multiple of FLASH_PAGE_SIZE (256 bytes).
    static uint8_t buf[FLASH_SECTOR_SIZE];
    memset(buf, 0xFF, sizeof(buf));
    memcpy(buf, &data, sizeof(MagCalibData));

    // Erase then write — interrupts must be disabled (RP2040 requirement)
    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_erase(MAG_CALIB_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(MAG_CALIB_FLASH_OFFSET, buf, FLASH_SECTOR_SIZE);
    restore_interrupts(irq_state);
}

void MagCalibStore::erase()
{
    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_erase(MAG_CALIB_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(irq_state);
}

// ── computeCRC() ──────────────────────────────────────────────────────────────

uint16_t MagCalibStore::computeCRC(int16_t     hard_x,
                                    int16_t     hard_y,
                                    int16_t     hard_z,
                                    const float soft_iron[9])
{
    // XOR checksum — same algorithm as IMU CalibStore for consistency.
    // Covers all calibration fields: hard-iron offsets + soft-iron matrix.
    uint16_t crc = 0;

    crc ^= static_cast<uint16_t>(hard_x);
    crc ^= static_cast<uint16_t>(hard_y);
    crc ^= static_cast<uint16_t>(hard_z);

    // Soft-iron matrix — interpret each float's raw bytes as uint16 pairs
    // so the CRC covers the full bit pattern of every matrix element.
    for (int i = 0; i < 9; i++)
    {
        uint32_t bits;
        memcpy(&bits, &soft_iron[i], sizeof(float));
        crc ^= static_cast<uint16_t>(bits & 0xFFFF);
        crc ^= static_cast<uint16_t>(bits >> 16);
    }

    return crc;
}