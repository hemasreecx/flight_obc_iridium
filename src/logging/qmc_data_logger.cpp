
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "qmc_data_logger.hpp"
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
    if (_mode == MagLoggerMode::RAW)
    {
        // Raw counts — mirrors IMU RAW header style
        printf("raw_x,raw_y,raw_z,"
               "overflow,skipped,"
               "timestamp_ms\n");
    }
    else
    {
        // Converted — Gauss per axis, heading, temperature
        printf("x_gauss,y_gauss,z_gauss,"
               "heading_deg,temperature,"
               "overflow,skipped,"
               "timestamp_ms\n");
    }
}

// ── log() ─────────────────────────────────────────────────────────────────────

void MagDataLogger::log(const MagSample& sample) const
{
    if (_mode == MagLoggerMode::RAW)
    {
        // RAW mode: MagSample doesn't carry raw counts (calibration already
        // applied upstream). We print the Gauss values scaled back to counts
        // using the known scale factor so the CSV stays in integer-count units
        // matching what the sensor actually produced before conversion.
        //
        // Why not store raw counts in MagSample?
        // The acquisition pipeline calibrates before storing — keeping raw
        // counts too would double the struct size for a rarely-needed field.
        // If true pre-calibration counts are needed, use logRaw() directly
        // from the calibration routine instead.
        static constexpr float SCALE = 3000.0f;   // ±8G → 3000 LSB/Gauss

        printf("%d,%d,%d,%d,%d,%llu\n",
               (int)(sample.x_gauss * SCALE),
               (int)(sample.y_gauss * SCALE),
               (int)(sample.z_gauss * SCALE),
               (int)sample.overflow,
               (int)sample.data_skipped,
               sample.timestamp_ms);
    }
    else
    {
        // CONVERTED mode: Gauss values, heading, temperature — all processed
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
}

// ── logRaw() ──────────────────────────────────────────────────────────────────

void MagDataLogger::logRaw(int16_t x, int16_t y, int16_t z) const
{
    // Bypasses MagSample entirely — used during calibration routines
    // where you want to see raw sensor output before any processing.
    printf("%d,%d,%d\n", x, y, z);
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
    // ── Build the struct to write ─────────────────────────────────────────────
    MagCalibData data;
    data.magic  = MAG_CALIB_MAGIC;
    data.hard_x = hard_x;
    data.hard_y = hard_y;
    data.hard_z = hard_z;
    memcpy(data.soft_iron, soft_iron, sizeof(data.soft_iron));
    data.crc = computeCRC(hard_x, hard_y, hard_z, soft_iron);

    // ── Pad to full 4KB sector ────────────────────────────────────────────────
    // flash_range_program() requires the buffer to be a multiple of
    // FLASH_PAGE_SIZE (256 bytes). We use a full sector buffer (4096 bytes)
    // to be safe, zeroed out, with our struct at the front.
    static uint8_t buf[FLASH_SECTOR_SIZE];
    memset(buf, 0xFF, sizeof(buf));          // 0xFF = erased flash state
    memcpy(buf, &data, sizeof(MagCalibData));

    // ── Erase then write (interrupts disabled — RP2040 requirement) ───────────
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

    // Hard-iron offsets
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