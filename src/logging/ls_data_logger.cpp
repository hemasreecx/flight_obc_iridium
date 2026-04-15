#include "ls_data_logger.hpp"
#include <stdio.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

/* ============================================================
   LSM6DSV80X_DataLogger
   ============================================================ */

LSM6DSV80X_DataLogger::LSM6DSV80X_DataLogger(LSM6DSV80X_LoggerMode mode)
    : _mode(mode)
{
}

void LSM6DSV80X_DataLogger::setMode(LSM6DSV80X_LoggerMode mode)
{
    _mode = mode;
}

LSM6DSV80X_LoggerMode LSM6DSV80X_DataLogger::getMode() const
{
    return _mode;
}

void LSM6DSV80X_DataLogger::printHeader() const
{
    switch (_mode)
    {
        case LSM6DSV80X_LoggerMode::RAW:
            printf("hg_x_raw,hg_y_raw,hg_z_raw,"
                   "gy_x_raw,gy_y_raw,gy_z_raw,"
                   "temp_raw\n");
            break;

        case LSM6DSV80X_LoggerMode::CONVERTED:
            printf("hg_x[g],hg_y[g],hg_z[g],"
                   "gy_x[dps],gy_y[dps],gy_z[dps],"
                   "temp[C]\n");
            break;

        default:
            printf("hg_x[g],hg_y[g],hg_z[g],"
                   "gy_x[dps],gy_y[dps],gy_z[dps],"
                   "temp[C]\n");
            break;
    }
}

// printf requires double for %f — cast floats explicitly.
// Temperature conversion: temp_celsius = (raw / 256.0f) + 25.0f
// The offset has already been subtracted from raw_temp upstream
// in LSM6DSV80X_Acquisition::task() before passing here.
void LSM6DSV80X_DataLogger::log(const LSM6DSV80X_RawGY& raw_gy,
                                 const LSM6DSV80X_RawHG& raw_hg,
                                 int16_t                 raw_temp,
                                 const LSM6DSV80X_Data&  conv_gy,
                                 const LSM6DSV80X_Data&  conv_hg) const
{
    switch (_mode)
    {
        case LSM6DSV80X_LoggerMode::RAW:
            printf("%d,%d,%d,%d,%d,%d,%d\n",
                   raw_hg.x, raw_hg.y, raw_hg.z,
                   raw_gy.x, raw_gy.y, raw_gy.z,
                   raw_temp);
            break;

        case LSM6DSV80X_LoggerMode::CONVERTED:
        {
            float temp_c = (static_cast<float>(raw_temp) / 256.0f) + 25.0f;
            printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f\n",
                   (double)conv_hg.x, (double)conv_hg.y, (double)conv_hg.z,
                   (double)conv_gy.x, (double)conv_gy.y, (double)conv_gy.z,
                   (double)temp_c);
            break;
        }

        default:
            printf("0,0,0,0,0,0,0\n");
            break;
    }
}


/* ============================================================
   LSM6DSV80X_CalibStore
   ============================================================

   Flash memory map (RP2040, 2MB flash):
   ┌─────────────────────────────┐ 0x000000
   │   program code + data       │
   │   ... empty space ...       │
   ├─────────────────────────────┤ 0x1FF000  ← CALIB_FLASH_OFFSET
   │   LSM6DSV80X_CalibData      │  last 4KB sector
   └─────────────────────────────┘ 0x200000

   The Pico SDK flash functions take offsets from the START
   of flash (0x000000), not from XIP_BASE (0x10000000).
   Reading flash uses XIP_BASE + offset as a plain pointer.
   Writing uses the raw offset with flash_range_program().

   CRITICAL: flash_range_erase() and flash_range_program()
   must not run while executing from flash.
   save_and_disable_interrupts() handles this automatically.
   ============================================================ */

static const LSM6DSV80X_CalibData* flash_calib_ptr()
{
    return reinterpret_cast<const LSM6DSV80X_CalibData*>(
        XIP_BASE + LSM6DSV80X_CalibStore::CALIB_FLASH_OFFSET
    );
}

/* ============================================================
   computeCRC
   XOR checksum of all seven int16_t offsets.
   Adding temp_off to the CRC means any corruption of the
   temperature offset will be detected on next boot.
   ============================================================ */

uint16_t LSM6DSV80X_CalibStore::computeCRC(int16_t gy_x,   int16_t gy_y,   int16_t gy_z,
                                            int16_t hg_x,   int16_t hg_y,   int16_t hg_z,
                                            int16_t temp_off)
{
    return static_cast<uint16_t>(gy_x)    ^
           static_cast<uint16_t>(gy_y)    ^
           static_cast<uint16_t>(gy_z)    ^
           static_cast<uint16_t>(hg_x)    ^
           static_cast<uint16_t>(hg_y)    ^
           static_cast<uint16_t>(hg_z)    ^
           static_cast<uint16_t>(temp_off);
}

/* ============================================================
   load
   ============================================================ */

bool LSM6DSV80X_CalibStore::load(LSM6DSV80X_CalibData& data)
{
    const LSM6DSV80X_CalibData* stored = flash_calib_ptr();

    if (stored->magic != CALIB_MAGIC)
        return false;

    uint16_t expected = computeCRC(stored->gy_off_x, stored->gy_off_y, stored->gy_off_z,
                                   stored->hg_off_x, stored->hg_off_y, stored->hg_off_z,
                                   stored->temp_off);
    if (stored->crc != expected)
        return false;

    memcpy(&data, stored, sizeof(LSM6DSV80X_CalibData));
    return true;
}

/* ============================================================
   save
   ============================================================ */

void LSM6DSV80X_CalibStore::save(int16_t gy_x,   int16_t gy_y,   int16_t gy_z,
                                  int16_t hg_x,   int16_t hg_y,   int16_t hg_z,
                                  int16_t temp_off)
{
    static_assert(sizeof(LSM6DSV80X_CalibData) <= FLASH_PAGE_SIZE,
                  "LSM6DSV80X_CalibData exceeds one flash page — increase buffer size");

    LSM6DSV80X_CalibData data{};
    data.magic    = CALIB_MAGIC;
    data.gy_off_x = gy_x;
    data.gy_off_y = gy_y;
    data.gy_off_z = gy_z;
    data.hg_off_x = hg_x;
    data.hg_off_y = hg_y;
    data.hg_off_z = hg_z;
    data.temp_off = temp_off;
    data.crc      = computeCRC(gy_x, gy_y, gy_z, hg_x, hg_y, hg_z, temp_off);

    uint8_t page_buf[FLASH_PAGE_SIZE];
    memset(page_buf, 0xFF, sizeof(page_buf));
    memcpy(page_buf, &data, sizeof(LSM6DSV80X_CalibData));

    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase  (CALIB_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(CALIB_FLASH_OFFSET, page_buf, FLASH_PAGE_SIZE);
    restore_interrupts(irq);
}

/* ============================================================
   erase
   ============================================================ */

void LSM6DSV80X_CalibStore::erase()
{
    uint32_t irq = save_and_disable_interrupts();
    flash_range_erase(CALIB_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    restore_interrupts(irq);
}

/* ============================================================
   isValid
   ============================================================ */

bool LSM6DSV80X_CalibStore::isValid()
{
    const LSM6DSV80X_CalibData* stored = flash_calib_ptr();

    if (stored->magic != CALIB_MAGIC)
        return false;

    uint16_t expected = computeCRC(stored->gy_off_x, stored->gy_off_y, stored->gy_off_z,
                                   stored->hg_off_x, stored->hg_off_y, stored->hg_off_z,
                                   stored->temp_off);
    return (stored->crc == expected);
}