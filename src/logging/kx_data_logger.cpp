#include "kx_data_logger.hpp"
#include <stdio.h>
#include <string.h>                      // memcpy, memset

#include "hardware/flash.h"              // flash_range_erase, flash_range_program
#include "hardware/sync.h"               // save_and_disable_interrupts, restore_interrupts
#include "pico/stdlib.h"

/* ============================================================
   DataLogger
   ============================================================ */

DataLogger::DataLogger(LoggerMode mode)
    : _mode(mode)
{
}

void DataLogger::setMode(LoggerMode mode)
{
    _mode = mode;
}

LoggerMode DataLogger::getMode() const
{
    return _mode;
}

void DataLogger::printHeader() const
{
    switch (_mode)
    {
        case LoggerMode::RAW:
            printf("x_raw,y_raw,z_raw\n");
            break;

        case LoggerMode::CONVERTED:
            printf("x,y,z\n");
            break;

        default:
            printf("x,y,z\n");
            break;
    }
}

//      printf requires double for %f, not float directly.
void DataLogger::log(const KX134_Raw& raw,
                     const IMU_Data&  converted) const
{
    switch (_mode)
    {
        case LoggerMode::RAW:
            printf("%d,%d,%d\n",
                   raw.x,
                   raw.y,
                   raw.z);
            break;

        case LoggerMode::CONVERTED:
            printf("%.6f,%.6f,%.6f\n",
                   (double)converted.x_g,
                   (double)converted.y_g,
                   (double)converted.z_g);
            break;

        default:
            printf("0,0,0\n");
            break;
    }
}


/* ============================================================
   CalibStore
   ============================================================

   Flash memory map (RP2040, 2MB flash):
   ┌─────────────────────────────┐ 0x000000
   │   XIP base (not used here)  │
   ├─────────────────────────────┤ 0x100000  (XIP_BASE offset)
   │   program code + data       │
   │   (typically < 200KB)       │
   │                             │
   │   ... empty space ...       │
   │                             │
   ├─────────────────────────────┤ 0x1FF000  ← CALIB_FLASH_OFFSET
   │   CalibData (12 bytes)      │  last 4KB sector
   └─────────────────────────────┘ 0x200000

   The Pico SDK flash functions take offsets from the START
   of flash (0x000000), not from XIP_BASE (0x10000000).
   Reading flash uses XIP_BASE + offset as a plain pointer.
   Writing uses the raw offset with flash_range_program().

   CRITICAL: flash_range_erase() and flash_range_program()
   must not run while executing from flash.
   save_and_disable_interrupts() + restore_interrupts() handle
   this — the SDK runs these operations from RAM internally.
   ============================================================ */

// Flash is memory-mapped at XIP_BASE (0x10000000) for reading.
// Cast offset to pointer — reads CalibData directly, no SDK call.
static const CalibData* flash_calib_ptr()
{
    return reinterpret_cast<const CalibData*>(
        XIP_BASE + CalibStore::CALIB_FLASH_OFFSET
    );
}

/* ============================================================
   computeCRC
   XOR checksum of the three int16_t offsets.
   Catches single-bit flash corruption reliably.

   Example:
     off_x = 0x0004, off_y = 0x00E3, off_z = 0xFFD4
     crc   = 0x0004 ^ 0x00E3 ^ 0xFFD4 = 0xFF33
   ============================================================ */

uint16_t CalibStore::computeCRC(int16_t off_x,
                                 int16_t off_y,
                                 int16_t off_z)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(off_x) ^
        static_cast<uint16_t>(off_y) ^
        static_cast<uint16_t>(off_z)
    );
}

/* ============================================================
   load
   Reads CalibData from flash and validates it.

   Returns false if:
     - magic != CALIB_MAGIC  (blank flash reads as 0xFFFFFFFF)
     - CRC mismatch          (bit corruption detected)
   ============================================================ */

bool CalibStore::load(CalibData& data)
{
    // Read directly from flash via XIP memory map
    const CalibData* stored = flash_calib_ptr();

    // Step 1: check magic number
    if (stored->magic != CALIB_MAGIC)
        return false;

    // Step 2: recompute and compare CRC
    uint16_t expected_crc = computeCRC(stored->off_x,
                                        stored->off_y,
                                        stored->off_z);
    if (stored->crc != expected_crc)
        return false;

    // Valid — copy into caller's struct
    memcpy(&data, stored, sizeof(CalibData));
    return true;
}

/* ============================================================
   save
   Writes calibration offsets to flash.

   Steps:
     1. Build CalibData with magic + offsets + CRC
     2. Pad to FLASH_PAGE_SIZE (256 bytes) — SDK requirement
     3. Disable interrupts — flash write runs from RAM
     4. Erase 4KB sector   — bits can only go 1→0,
                              erase resets all bits to 1
     5. Program 256-byte page
     6. Restore interrupts
   ============================================================ */

void CalibStore::save(int16_t off_x,
                      int16_t off_y,
                      int16_t off_z)
{
    // Build struct
    CalibData data;
    data.magic = CALIB_MAGIC;
    data.off_x = off_x;
    data.off_y = off_y;
    data.off_z = off_z;
    data.crc   = computeCRC(off_x, off_y, off_z);

    // Pad to full page with 0xFF (erased flash state — neutral)
    uint8_t page_buf[FLASH_PAGE_SIZE];
    memset(page_buf, 0xFF, sizeof(page_buf));
    memcpy(page_buf, &data, sizeof(CalibData));

    // Disable interrupts — required before flash operations
    uint32_t irq = save_and_disable_interrupts();

    flash_range_erase(CALIB_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CALIB_FLASH_OFFSET, page_buf, FLASH_PAGE_SIZE);

    restore_interrupts(irq);
}


void CalibStore::erase()
{
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(CALIB_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(irq);
}