#include "LSM6DSV80X.hpp"
#include "hardware/i2c.h"
#include <cstring>
#include "pico/time.h"
#if LSM6DSV80X_DEBUG
#include <cstdio>
#define DBG(fmt, ...) printf("[LSM6DSV80X] " fmt "\n", ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

// ─── Bit Masks ────────────────────────────────────────────────────────────────

// CTRL1 (XL): odr_xl[3:0] = bits[3:0],  op_mode_xl[2:0] = bits[6:4]
#define CTRL1_ODR_XL_MASK       0x0F
#define CTRL1_ODR_XL_SHIFT      0
#define CTRL1_MODE_XL_MASK      0x70
#define CTRL1_MODE_XL_SHIFT     4

// CTRL2 (GY): odr_g[3:0] = bits[3:0],   op_mode_g[2:0] = bits[6:4]
#define CTRL2_ODR_GY_MASK       0x0F
#define CTRL2_ODR_GY_SHIFT      0
#define CTRL2_MODE_GY_MASK      0x70
#define CTRL2_MODE_GY_SHIFT     4

// CTRL3: sw_reset=bit0,  if_inc=bit2,  bdu=bit6,  boot=bit7
#define CTRL3_SW_RESET          0x01
#define CTRL3_IF_INC            0x04
#define CTRL3_BDU               0x40
#define CTRL3_BOOT              0x80

// CTRL6 (GY full-scale): fs_g[2:0] = bits[2:0]
#define CTRL6_FS_GY_MASK        0x07
#define CTRL6_FS_GY_SHIFT       0

// CTRL8 (XL full-scale): fs_xl[1:0] = bits[1:0]
#define CTRL8_FS_XL_MASK        0x03
#define CTRL8_FS_XL_SHIFT       0

// CTRL1_XL_HG (0x4E): fs_xl_hg[2:0]=bits[2:0], odr_xl_hg[2:0]=bits[5:3], xl_hg_regout_en=bit7
#define CTRL1_XL_HG_FS_MASK     0x07
#define CTRL1_XL_HG_FS_SHIFT    0
#define CTRL1_XL_HG_ODR_MASK    0x38
#define CTRL1_XL_HG_ODR_SHIFT   3
#define CTRL1_XL_HG_REGOUT      0x80

// CTRL2_XL_HG (0x4D): op_mode_xl_hg[2:0] = bits[6:4]
#define CTRL2_XL_HG_MODE_MASK   0x70
#define CTRL2_XL_HG_MODE_SHIFT  4

// STATUS_REG: xlda=bit0, gda=bit1, tda=bit2, xlhgda=bit3
#define STATUS_XLDA             0x01
#define STATUS_GDA              0x02
#define STATUS_TDA              0x04
#define STATUS_XLHGDA           0x08

// FIFO_STATUS2: fifo_wtm_ia=bit7, fifo_ovr_ia=bit6, fifo_full_ia=bit5, diff_fifo(msb)=bit0
#define FIFO_WTM_IA             0x80
#define FIFO_OVR_IA             0x40
#define FIFO_FULL_IA            0x20
#define FIFO_STATUS2_DIFF_MSB   0x01

// FIFO_CTRL4: fifo_mode[2:0] = bits[2:0]
#define FIFO_CTRL4_MODE_MASK    0x07

// FUNCTIONS_ENABLE: timestamp_en=bit6, interrupts_enable=bit7
#define FUNC_EN_TIMESTAMP       0x40
#define FUNC_EN_INTERRUPTS      0x80

// INT1_CTRL: int1_drdy_xl=bit0, int1_drdy_g=bit1
#define INT1_DRDY_XL            0x01
#define INT1_DRDY_GY            0x02

// CTRL10: st_xl[1:0]=bits[1:0], st_g[3:2]=bits[3:2]
#define CTRL10_ST_XL_MASK       0x03
#define CTRL10_ST_GY_MASK       0x0C

constexpr int      LSM_MAX_RETRIES = 3;
constexpr uint32_t LSM_TIMEOUT_US  = 5000;   // 5ms — safe margin for 400kHz I2C

// ─── Constructor ──────────────────────────────────────────────────────────────

LSM6DSV80X::LSM6DSV80X(i2c_inst_t* i2c, uint8_t address)
    : _i2c(i2c), _address(address)
{
}

// ─── Lifecycle ────────────────────────────────────────────────────────────────

LSM6DSV80X_Status LSM6DSV80X::init(const LSM6DSV80X_Config& config)
{
    // 1. Verify device identity
    LSM6DSV80X_Status s = checkID();
    if (s != LSM6DSV80X_Status::OK) return s;

    // 2. Software reset and wait for it to clear
    s = reset();
    if (s != LSM6DSV80X_Status::OK) return s;

    // 3. Enable register auto-increment (required for multi-byte reads)
    if (!modifyRegister(LSM6DSV80X_Register::CTRL3, CTRL3_IF_INC, CTRL3_IF_INC))
        return LSM6DSV80X_Status::ERR_COMM;

    // 4. BDU — prevents reading a stale mix of old-low/new-high bytes
    if (config.enable_bdu)
    {
        if (!modifyRegister(LSM6DSV80X_Register::CTRL3, CTRL3_BDU, CTRL3_BDU))
            return LSM6DSV80X_Status::ERR_COMM;
    }

    // 5. Accelerometer: full-scale first (CTRL8), then ODR + mode (CTRL1)
    s = setXLRange(config.xl_range);
    if (s != LSM6DSV80X_Status::OK) return s;

    s = setXLMode(config.xl_mode);
    if (s != LSM6DSV80X_Status::OK) return s;

    s = setXLODR(config.xl_odr);       // ODR last — starts the sensor
    if (s != LSM6DSV80X_Status::OK) return s;

    // 6. Gyroscope: full-scale first (CTRL6), then ODR + mode (CTRL2)
    s = setGYRange(config.gy_range);
    if (s != LSM6DSV80X_Status::OK) return s;

    s = setGYMode(config.gy_mode);
    if (s != LSM6DSV80X_Status::OK) return s;

    s = setGYODR(config.gy_odr);       // ODR last — starts the sensor
    if (s != LSM6DSV80X_Status::OK) return s;

    // 7. High-G accelerometer (optional)
    if (config.enable_hg_xl)
    {
        // Step 1: Set range first
        s = setHGRange(config.hg_range);
        if (s != LSM6DSV80X_Status::OK) return s;

        // Step 2: Set operating mode
        s = setHGMode(config.hg_mode);
        if (s != LSM6DSV80X_Status::OK) return s;

        // Step 3: Small settle delay before enabling ODR
        // Raw HG data path is controlled by CTRL1_XL_HG (ODR + regout_en).
        sleep_ms(5);

        // Step 4: Enable ODR + regout last — this starts the HG channel
        // reg_out_en=true routes HG data to UI_OUTX/Y/Z_L/H_A_HG registers
        s = setHGODR(config.hg_odr, true);
        if (s != LSM6DSV80X_Status::OK) return s;

        // Step 5: Wait for HG settling (CTRL_STATUS bit1 xl_hg_settling clears)
        uint8_t ctrl_status = 0;
        uint32_t tries = 100;
        do {
            sleep_ms(2);
            readRegister(LSM6DSV80X_Register::CTRL_STATUS, ctrl_status);
        } while ((ctrl_status & 0x02) && --tries);

        DBG("HG settling done (tries left=%lu, CTRL_STATUS=0x%02X)", tries, ctrl_status);
    }

    // 8. Timestamp counter
    if (config.enable_timestamp)
    {
        if (!modifyRegister(LSM6DSV80X_Register::FUNCTIONS_ENABLE,
                            FUNC_EN_TIMESTAMP | FUNC_EN_INTERRUPTS,
                            FUNC_EN_TIMESTAMP | FUNC_EN_INTERRUPTS))
            return LSM6DSV80X_Status::ERR_COMM;
    }

    // 9. Data-ready interrupt routed to INT1
    if (config.enable_drdy_int1)
    {
        if (!modifyRegister(LSM6DSV80X_Register::INT1_CTRL,
                            INT1_DRDY_XL | INT1_DRDY_GY,
                            INT1_DRDY_XL | INT1_DRDY_GY))
            return LSM6DSV80X_Status::ERR_COMM;
    }

    // 10. FIFO — BYPASS is the reset default, but be explicit
    if (!config.enable_fifo)
    {
        if (!modifyRegister(LSM6DSV80X_Register::FIFO_CTRL4,
                            FIFO_CTRL4_MODE_MASK,
                            static_cast<uint8_t>(LSM6DSV80X_FIFO_Mode::BYPASS)))
            return LSM6DSV80X_Status::ERR_COMM;
    }
    else
    {
        s = setFIFOWatermark(config.fifo_watermark);
        if (s != LSM6DSV80X_Status::OK) return s;

        s = setFIFOMode(config.fifo_mode);
        if (s != LSM6DSV80X_Status::OK) return s;
    }

    DBG("init complete");
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::reset()
{
    if (!modifyRegister(LSM6DSV80X_Register::CTRL3, CTRL3_SW_RESET, CTRL3_SW_RESET))
        return LSM6DSV80X_Status::ERR_COMM;

    uint8_t  val   = 0;
    uint32_t tries = 100;
    do {
        sleep_ms(2);
        if (!readRegister(LSM6DSV80X_Register::CTRL3, val))
            return LSM6DSV80X_Status::ERR_COMM;
    } while ((val & CTRL3_SW_RESET) && --tries);

    if (tries == 0)
    {
        DBG("reset timeout");
        return LSM6DSV80X_Status::ERR_TIMEOUT;
    }

    DBG("reset OK");
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::checkID()
{
    uint8_t id = 0;
    if (!readRegister(LSM6DSV80X_Register::WHO_AM_I, id))
        return LSM6DSV80X_Status::ERR_COMM;

    if (id != LSM6DSV80X_WHO_AM_I_VALUE)
    {
        DBG("WHO_AM_I mismatch: got 0x%02X, expected 0x%02X", id, LSM6DSV80X_WHO_AM_I_VALUE);
        return LSM6DSV80X_Status::ERR_INVALID_ID;
    }

    DBG("WHO_AM_I OK (0x%02X)", id);
    return LSM6DSV80X_Status::OK;
}

// ─── Low-G Accelerometer ──────────────────────────────────────────────────────

LSM6DSV80X_Status LSM6DSV80X::setXLRange(LSM6DSV80X_XL_Range range)
{
    uint8_t val = static_cast<uint8_t>(range) << CTRL8_FS_XL_SHIFT;
    if (!modifyRegister(LSM6DSV80X_Register::CTRL8, CTRL8_FS_XL_MASK, val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::getXLRange(LSM6DSV80X_XL_Range& range)
{
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::CTRL8, val))
        return LSM6DSV80X_Status::ERR_COMM;
    range = static_cast<LSM6DSV80X_XL_Range>((val >> CTRL8_FS_XL_SHIFT) & CTRL8_FS_XL_MASK);
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::setXLODR(LSM6DSV80X_ODR odr)
{
    uint8_t val = static_cast<uint8_t>(odr) << CTRL1_ODR_XL_SHIFT;
    if (!modifyRegister(LSM6DSV80X_Register::CTRL1, CTRL1_ODR_XL_MASK, val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::getXLODR(LSM6DSV80X_ODR& odr)
{
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::CTRL1, val))
        return LSM6DSV80X_Status::ERR_COMM;
    odr = static_cast<LSM6DSV80X_ODR>((val >> CTRL1_ODR_XL_SHIFT) & CTRL1_ODR_XL_MASK);
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::setXLMode(LSM6DSV80X_XL_Mode mode)
{
    uint8_t val = static_cast<uint8_t>(mode) << CTRL1_MODE_XL_SHIFT;
    if (!modifyRegister(LSM6DSV80X_Register::CTRL1, CTRL1_MODE_XL_MASK, val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::getXLMode(LSM6DSV80X_XL_Mode& mode)
{
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::CTRL1, val))
        return LSM6DSV80X_Status::ERR_COMM;
    mode = static_cast<LSM6DSV80X_XL_Mode>((val >> CTRL1_MODE_XL_SHIFT) & 0x07);
    return LSM6DSV80X_Status::OK;
}

// ─── Gyroscope ────────────────────────────────────────────────────────────────

LSM6DSV80X_Status LSM6DSV80X::setGYRange(LSM6DSV80X_GY_Range range)
{
    uint8_t val = static_cast<uint8_t>(range) << CTRL6_FS_GY_SHIFT;
    if (!modifyRegister(LSM6DSV80X_Register::CTRL6, CTRL6_FS_GY_MASK, val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::getGYRange(LSM6DSV80X_GY_Range& range)
{
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::CTRL6, val))
        return LSM6DSV80X_Status::ERR_COMM;
    range = static_cast<LSM6DSV80X_GY_Range>((val >> CTRL6_FS_GY_SHIFT) & CTRL6_FS_GY_MASK);
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::setGYODR(LSM6DSV80X_ODR odr)
{
    uint8_t val = static_cast<uint8_t>(odr) << CTRL2_ODR_GY_SHIFT;
    if (!modifyRegister(LSM6DSV80X_Register::CTRL2, CTRL2_ODR_GY_MASK, val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::getGYODR(LSM6DSV80X_ODR& odr)
{
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::CTRL2, val))
        return LSM6DSV80X_Status::ERR_COMM;
    odr = static_cast<LSM6DSV80X_ODR>((val >> CTRL2_ODR_GY_SHIFT) & CTRL2_ODR_GY_MASK);
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::setGYMode(LSM6DSV80X_GY_Mode mode)
{
    uint8_t val = static_cast<uint8_t>(mode) << CTRL2_MODE_GY_SHIFT;
    if (!modifyRegister(LSM6DSV80X_Register::CTRL2, CTRL2_MODE_GY_MASK, val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::getGYMode(LSM6DSV80X_GY_Mode& mode)
{
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::CTRL2, val))
        return LSM6DSV80X_Status::ERR_COMM;
    mode = static_cast<LSM6DSV80X_GY_Mode>((val >> CTRL2_MODE_GY_SHIFT) & 0x07);
    return LSM6DSV80X_Status::OK;
}

// ─── High-G Accelerometer ─────────────────────────────────────────────────────

LSM6DSV80X_Status LSM6DSV80X::setHGRange(LSM6DSV80X_HG_Range range)
{
    // CTRL1_XL_HG (0x4E) bits[2:0] = fs_xl_hg
    uint8_t val = static_cast<uint8_t>(range) << CTRL1_XL_HG_FS_SHIFT;
    if (!modifyRegister(LSM6DSV80X_Register::CTRL1_XL_HG, CTRL1_XL_HG_FS_MASK, val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::getHGRange(LSM6DSV80X_HG_Range& range)
{
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::CTRL1_XL_HG, val))
        return LSM6DSV80X_Status::ERR_COMM;
    range = static_cast<LSM6DSV80X_HG_Range>((val >> CTRL1_XL_HG_FS_SHIFT) & CTRL1_XL_HG_FS_MASK);
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::setHGODR(LSM6DSV80X_HG_ODR odr, bool reg_out_en)
{
    // CTRL1_XL_HG bits[5:3]=odr_xl_hg, bit7=xl_hg_regout_en
    uint8_t odr_val    = static_cast<uint8_t>(odr) << CTRL1_XL_HG_ODR_SHIFT;
    uint8_t regout_val = reg_out_en ? CTRL1_XL_HG_REGOUT : 0x00;
    uint8_t mask       = CTRL1_XL_HG_ODR_MASK | CTRL1_XL_HG_REGOUT;

    if (!modifyRegister(LSM6DSV80X_Register::CTRL1_XL_HG, mask, odr_val | regout_val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::getHGODR(LSM6DSV80X_HG_ODR& odr, bool& reg_out_en)
{
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::CTRL1_XL_HG, val))
        return LSM6DSV80X_Status::ERR_COMM;
    odr        = static_cast<LSM6DSV80X_HG_ODR>((val & CTRL1_XL_HG_ODR_MASK) >> CTRL1_XL_HG_ODR_SHIFT);
    reg_out_en = (val & CTRL1_XL_HG_REGOUT) != 0;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::setHGMode(LSM6DSV80X_HG_Mode mode)
{
    // CTRL2_XL_HG (0x4D) bits[6:4] = op_mode_xl_hg
    uint8_t val = static_cast<uint8_t>(mode) << CTRL2_XL_HG_MODE_SHIFT;
    if (!modifyRegister(LSM6DSV80X_Register::CTRL2_XL_HG, CTRL2_XL_HG_MODE_MASK, val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::getHGMode(LSM6DSV80X_HG_Mode& mode)
{
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::CTRL2_XL_HG, val))
        return LSM6DSV80X_Status::ERR_COMM;
    mode = static_cast<LSM6DSV80X_HG_Mode>((val & CTRL2_XL_HG_MODE_MASK) >> CTRL2_XL_HG_MODE_SHIFT);
    return LSM6DSV80X_Status::OK;
}

// ─── Data Reads ───────────────────────────────────────────────────────────────

LSM6DSV80X_Status LSM6DSV80X::readRawXL(LSM6DSV80X_RawXL& raw)
{
    uint8_t buf[6] = {0};
    if (!readMulti(LSM6DSV80X_Register::OUTX_L_A, buf, 6))
        return LSM6DSV80X_Status::ERR_COMM;

    raw.x = combine(buf[0], buf[1]);
    raw.y = combine(buf[2], buf[3]);
    raw.z = combine(buf[4], buf[5]);
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::readRawGY(LSM6DSV80X_RawGY& raw)
{
    uint8_t buf[6] = {0};
    if (!readMulti(LSM6DSV80X_Register::OUTX_L_G, buf, 6))
        return LSM6DSV80X_Status::ERR_COMM;

    raw.x = combine(buf[0], buf[1]);
    raw.y = combine(buf[2], buf[3]);
    raw.z = combine(buf[4], buf[5]);
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::readRawHG(LSM6DSV80X_RawHG& raw)
{
    // HG output lives in UI_OUTX/Y/Z_L/H_A_HG (0x34–0x39)
    uint8_t buf[6] = {0};
    if (!readMulti(LSM6DSV80X_Register::UI_OUTX_L_A_HG, buf, 6))
        return LSM6DSV80X_Status::ERR_COMM;

    raw.x = combine(buf[0], buf[1]);
    raw.y = combine(buf[2], buf[3]);
    raw.z = combine(buf[4], buf[5]);
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::readTemperature(int16_t& raw_temp)
{
    uint8_t buf[2] = {0};
    if (!readMulti(LSM6DSV80X_Register::OUT_TEMP_L, buf, 2))
        return LSM6DSV80X_Status::ERR_COMM;

    raw_temp = combine(buf[0], buf[1]);
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::readTimestamp(uint32_t& timestamp)
{
    uint8_t buf[4] = {0};
    if (!readMulti(LSM6DSV80X_Register::TIMESTAMP0, buf, 4))
        return LSM6DSV80X_Status::ERR_COMM;

    // 32-bit little-endian; 1 LSB = 25 µs
    timestamp = static_cast<uint32_t>(buf[0])
              | (static_cast<uint32_t>(buf[1]) << 8)
              | (static_cast<uint32_t>(buf[2]) << 16)
              | (static_cast<uint32_t>(buf[3]) << 24);
    return LSM6DSV80X_Status::OK;
}

// ─── FIFO ─────────────────────────────────────────────────────────────────────

LSM6DSV80X_Status LSM6DSV80X::setFIFOMode(LSM6DSV80X_FIFO_Mode mode)
{
    uint8_t val = static_cast<uint8_t>(mode) & FIFO_CTRL4_MODE_MASK;
    if (!modifyRegister(LSM6DSV80X_Register::FIFO_CTRL4, FIFO_CTRL4_MODE_MASK, val))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

LSM6DSV80X_Status LSM6DSV80X::setFIFOWatermark(uint8_t level)
{
    if (!writeRegister(LSM6DSV80X_Register::FIFO_CTRL1, level))
        return LSM6DSV80X_Status::ERR_COMM;
    return LSM6DSV80X_Status::OK;
}

uint16_t LSM6DSV80X::readFIFOLevel()
{
    uint8_t s1 = 0, s2 = 0;
    if (!readRegister(LSM6DSV80X_Register::FIFO_STATUS1, s1)) return 0;
    if (!readRegister(LSM6DSV80X_Register::FIFO_STATUS2, s2)) return 0;

    return static_cast<uint16_t>(s1)
         | (static_cast<uint16_t>(s2 & FIFO_STATUS2_DIFF_MSB) << 8);
}

LSM6DSV80X_Status LSM6DSV80X::readFIFOSample(LSM6DSV80X_FifoSample& sample)
{
    uint8_t tag_byte = 0;
    if (!readRegister(LSM6DSV80X_Register::FIFO_DATA_OUT_TAG, tag_byte))
        return LSM6DSV80X_Status::ERR_COMM;

    sample.tag = (tag_byte >> 3) & 0x1F;
    sample.cnt = (tag_byte >> 1) & 0x03;

    if (!readMulti(LSM6DSV80X_Register::FIFO_DATA_OUT_X_L, sample.data, 6))
        return LSM6DSV80X_Status::ERR_COMM;

    return LSM6DSV80X_Status::OK;
}

uint16_t LSM6DSV80X::readFIFO(LSM6DSV80X_FifoSample* buffer, uint16_t max_samples)
{
    uint16_t level = readFIFOLevel();
    uint16_t count = (level < max_samples) ? level : max_samples;

    for (uint16_t i = 0; i < count; i++)
    {
        if (readFIFOSample(buffer[i]) != LSM6DSV80X_Status::OK)
            return i;
    }
    return count;
}

// ─── Status / Interrupts ──────────────────────────────────────────────────────

LSM6DSV80X_DataReady LSM6DSV80X::dataReady()
{
    LSM6DSV80X_DataReady dr = {false, false, false, false};
    uint8_t val = 0;
    if (!readRegister(LSM6DSV80X_Register::STATUS_REG, val)) return dr;

    dr.xl    = (val & STATUS_XLDA)   != 0;
    dr.gy    = (val & STATUS_GDA)    != 0;
    dr.temp  = (val & STATUS_TDA)    != 0;
    dr.hg_xl = (val & STATUS_XLHGDA) != 0;
    return dr;
}

bool LSM6DSV80X::fifoWatermark()
{
    uint8_t val = 0;
    readRegister(LSM6DSV80X_Register::FIFO_STATUS2, val);
    return (val & FIFO_WTM_IA) != 0;
}

bool LSM6DSV80X::fifoFull()
{
    uint8_t val = 0;
    readRegister(LSM6DSV80X_Register::FIFO_STATUS2, val);
    return (val & FIFO_FULL_IA) != 0;
}

bool LSM6DSV80X::fifoOverrun()
{
    uint8_t val = 0;
    readRegister(LSM6DSV80X_Register::FIFO_STATUS2, val);
    return (val & FIFO_OVR_IA) != 0;
}

// ─── Self-Test ────────────────────────────────────────────────────────────────

LSM6DSV80X_Status LSM6DSV80X::selfTestXL()
{
    if (!modifyRegister(LSM6DSV80X_Register::CTRL10, CTRL10_ST_XL_MASK, 0x01))
        return LSM6DSV80X_Status::ERR_COMM;

    sleep_ms(100);

    LSM6DSV80X_RawXL st_raw;
    LSM6DSV80X_Status s = readRawXL(st_raw);

    modifyRegister(LSM6DSV80X_Register::CTRL10, CTRL10_ST_XL_MASK, 0x00);
    return s;
}

LSM6DSV80X_Status LSM6DSV80X::selfTestGY()
{
    if (!modifyRegister(LSM6DSV80X_Register::CTRL10, CTRL10_ST_GY_MASK, 0x04))
        return LSM6DSV80X_Status::ERR_COMM;

    sleep_ms(100);

    LSM6DSV80X_RawGY st_raw;
    LSM6DSV80X_Status s = readRawGY(st_raw);

    modifyRegister(LSM6DSV80X_Register::CTRL10, CTRL10_ST_GY_MASK, 0x00);
    return s;
}

// ─── debugRegisters ───────────────────────────────────────────────────────────
// Reads and prints raw values of all HG-related registers.
// Used to diagnose why HG output registers stay zero.

void LSM6DSV80X::debugRegisters()
{
#if LSM6DSV80X_DEBUG
    uint8_t val = 0;

    // CTRL1_XL_HG (0x4E) — ODR, FS, regout_en
    readRegister(LSM6DSV80X_Register::CTRL1_XL_HG, val);
    printf("[DBG] CTRL1_XL_HG (0x4E) = 0x%02X\n", val);

    // CTRL2_XL_HG (0x4D) — mode bits
    readRegister(LSM6DSV80X_Register::CTRL2_XL_HG, val);
    printf("[DBG] CTRL2_XL_HG (0x4D) = 0x%02X\n", val);

    // HG_FUNCTIONS_ENABLE (0x52)
    readRegister(LSM6DSV80X_Register::HG_FUNCTIONS_ENABLE, val);
    printf("[DBG] HG_FUNCTIONS_ENABLE (0x52) = 0x%02X\n", val);

    // CTRL_STATUS (0x1A) — xl_hg_settling bit1
    readRegister(LSM6DSV80X_Register::CTRL_STATUS, val);
    printf("[DBG] CTRL_STATUS (0x1A) = 0x%02X  (bit1=xl_hg_settling)\n", val);

    // STATUS_REG (0x1E)
    readRegister(LSM6DSV80X_Register::STATUS_REG, val);
    printf("[DBG] STATUS_REG  (0x1E) = 0x%02X  (bit3=xlhgda)\n", val);

    // Raw bytes at UI_OUTX_L_A_HG (0x34)
    uint8_t buf[6] = {0};
    readMulti(LSM6DSV80X_Register::UI_OUTX_L_A_HG, buf, 6);
    printf("[DBG] UI_OUT_HG bytes: %02X %02X %02X %02X %02X %02X\n",
           buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

    // FUNCTIONS_ENABLE (0x50)
    readRegister(LSM6DSV80X_Register::FUNCTIONS_ENABLE, val);
    printf("[DBG] FUNCTIONS_ENABLE (0x50) = 0x%02X\n", val);
#endif
}

bool LSM6DSV80X::writeRegister(LSM6DSV80X_Register reg, uint8_t value)
{
    uint8_t buf[2] = { static_cast<uint8_t>(reg), value };

    for (int i = 0; i < LSM_MAX_RETRIES; i++)
    {
        absolute_time_t t = make_timeout_time_us(LSM_TIMEOUT_US);
        if (i2c_write_blocking_until(_i2c, _address, buf, 2, false, t) == 2)
            return true;
        sleep_us(100);
    }
    return false;
}

bool LSM6DSV80X::readRegister(LSM6DSV80X_Register reg, uint8_t& value)
{
    uint8_t r = static_cast<uint8_t>(reg);

    for (int i = 0; i < LSM_MAX_RETRIES; i++)
    {
        absolute_time_t t1 = make_timeout_time_us(LSM_TIMEOUT_US);
        if (i2c_write_blocking_until(_i2c, _address, &r, 1, true, t1) == 1)
        {
            absolute_time_t t2 = make_timeout_time_us(LSM_TIMEOUT_US);
            if (i2c_read_blocking_until(_i2c, _address, &value, 1, false, t2) == 1)
                return true;
        }
    }
    return false;
}

bool LSM6DSV80X::readMulti(LSM6DSV80X_Register reg, uint8_t* buffer, uint16_t length)
{
    uint8_t r = static_cast<uint8_t>(reg);

    for (int i = 0; i < LSM_MAX_RETRIES; i++)
    {
        absolute_time_t t1 = make_timeout_time_us(LSM_TIMEOUT_US);
        if (i2c_write_blocking_until(_i2c, _address, &r, 1, true, t1) == 1)
        {
            absolute_time_t t2 = make_timeout_time_us(LSM_TIMEOUT_US);
            if (i2c_read_blocking_until(_i2c, _address, buffer, length, false, t2)
                == static_cast<int>(length))
                return true;
        }
    }
    return false;
}

bool LSM6DSV80X::modifyRegister(LSM6DSV80X_Register reg, uint8_t mask, uint8_t value)
{
    uint8_t current = 0;
    if (!readRegister(reg, current)) return false;
    uint8_t updated = (current & ~mask) | (value & mask);
    return writeRegister(reg, updated);
}

int16_t LSM6DSV80X::combine(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}