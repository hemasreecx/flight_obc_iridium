#ifndef LSM6DSV80X_HPP
#define LSM6DSV80X_HPP

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

#ifndef LSM6DSV80X_DEBUG
#define LSM6DSV80X_DEBUG 1
#endif

// Enable/disable low-G accelerometer path at compile time
#define LSM6DSV80X_ENABLE_LOW_G  0   // Set to 1 to re-enable low-G


// ─── Register Map ─────────────────────────────────────────────────────────────

enum class LSM6DSV80X_Register : uint8_t
{
    FUNC_CFG_ACCESS         = 0x01,
    PIN_CTRL                = 0x02,
    IF_CFG                  = 0x03,
    ODR_TRIG_CFG            = 0x06,
    FIFO_CTRL1              = 0x07,
    FIFO_CTRL2              = 0x08,
    FIFO_CTRL3              = 0x09,
    FIFO_CTRL4              = 0x0A,
    COUNTER_BDR_REG1        = 0x0B,
    COUNTER_BDR_REG2        = 0x0C,
    INT1_CTRL               = 0x0D,
    INT2_CTRL               = 0x0E,
    WHO_AM_I                = 0x0F,
    CTRL1                   = 0x10,
    CTRL2                   = 0x11,
    CTRL3                   = 0x12,
    CTRL4                   = 0x13,
    CTRL5                   = 0x14,
    CTRL6                   = 0x15,
    CTRL7                   = 0x16,
    CTRL8                   = 0x17,
    CTRL9                   = 0x18,
    CTRL10                  = 0x19,
    CTRL_STATUS             = 0x1A,
    FIFO_STATUS1            = 0x1B,
    FIFO_STATUS2            = 0x1C,
    ALL_INT_SRC             = 0x1D,
    STATUS_REG              = 0x1E,
    OUT_TEMP_L              = 0x20,
    OUT_TEMP_H              = 0x21,
    OUTX_L_G                = 0x22,
    OUTX_H_G                = 0x23,
    OUTY_L_G                = 0x24,
    OUTY_H_G                = 0x25,
    OUTZ_L_G                = 0x26,
    OUTZ_H_G                = 0x27,
    OUTX_L_A                = 0x28,
    OUTX_H_A                = 0x29,
    OUTY_L_A                = 0x2A,
    OUTY_H_A                = 0x2B,
    OUTZ_L_A                = 0x2C,
    OUTZ_H_A                = 0x2D,
    UI_OUTX_L_A_HG          = 0x34,
    UI_OUTX_H_A_HG          = 0x35,
    UI_OUTY_L_A_HG          = 0x36,
    UI_OUTY_H_A_HG          = 0x37,
    UI_OUTZ_L_A_HG          = 0x38,
    UI_OUTZ_H_A_HG          = 0x39,
    TIMESTAMP0              = 0x40,
    TIMESTAMP1              = 0x41,
    TIMESTAMP2              = 0x42,
    TIMESTAMP3              = 0x43,
    WAKE_UP_SRC             = 0x45,
    TAP_SRC                 = 0x46,
    D6D_SRC                 = 0x47,
    HG_WAKE_UP_SRC          = 0x4C,
    CTRL2_XL_HG             = 0x4D,   // HG self-test/offset-correction controls
    CTRL1_XL_HG             = 0x4E,   // ODR/FS/regout for high-G accel
    INTERNAL_FREQ           = 0x4F,
    FUNCTIONS_ENABLE        = 0x50,
    HG_FUNCTIONS_ENABLE     = 0x52,
    HG_WAKE_UP_THS          = 0x53,
    INACTIVITY_DUR          = 0x54,
    INACTIVITY_THS          = 0x55,
    TAP_CFG0                = 0x56,
    TAP_CFG1                = 0x57,
    TAP_CFG2                = 0x58,
    TAP_THS_6D              = 0x59,
    TAP_DUR                 = 0x5A,
    WAKE_UP_THS             = 0x5B,
    WAKE_UP_DUR             = 0x5C,
    FREE_FALL               = 0x5D,
    MD1_CFG                 = 0x5E,
    MD2_CFG                 = 0x5F,
    EMB_FUNC_CFG            = 0x63,
    X_OFS_USR               = 0x73,
    Y_OFS_USR               = 0x74,
    Z_OFS_USR               = 0x75,
    FIFO_DATA_OUT_TAG       = 0x78,
    FIFO_DATA_OUT_X_L       = 0x79,
    FIFO_DATA_OUT_X_H       = 0x7A,
    FIFO_DATA_OUT_Y_L       = 0x7B,
    FIFO_DATA_OUT_Y_H       = 0x7C,
    FIFO_DATA_OUT_Z_L       = 0x7D,
    FIFO_DATA_OUT_Z_H       = 0x7E
};

// ─── Enums ────────────────────────────────────────────────────────────────────

enum class LSM6DSV80X_XL_Range : uint8_t
{
    RANGE_2G  = 0x00,
    RANGE_4G  = 0x01,
    RANGE_8G  = 0x02,
    RANGE_16G = 0x03
};

enum class LSM6DSV80X_HG_Range : uint8_t
{
    RANGE_32G = 0x00,
    RANGE_64G = 0x01,
    RANGE_80G = 0x02
};

enum class LSM6DSV80X_GY_Range : uint8_t
{
    RANGE_125DPS  = 0x01,
    RANGE_250DPS  = 0x02,
    RANGE_500DPS  = 0x03,
    RANGE_1000DPS = 0x04,
    RANGE_2000DPS = 0x05,
    RANGE_4000DPS = 0x06
};

enum class LSM6DSV80X_ODR : uint8_t
{
    ODR_OFF       = 0x00,
    ODR_1HZ875    = 0x01,
    ODR_7HZ5      = 0x02,
    ODR_15HZ      = 0x03,
    ODR_30HZ      = 0x04,
    ODR_60HZ      = 0x05,
    ODR_120HZ     = 0x06,
    ODR_240HZ     = 0x07,
    ODR_480HZ     = 0x08,
    ODR_960HZ     = 0x09,
    ODR_1920HZ    = 0x0A,
    ODR_3840HZ    = 0x0B,
    ODR_7680HZ    = 0x0C
};

// HG accelerometer ODR — minimum is 480 Hz, no lower rates exist on this channel
enum class LSM6DSV80X_HG_ODR : uint8_t
{
    HG_ODR_OFF    = 0x00,
    HG_ODR_480HZ  = 0x03,   // Minimum available — use this as default
    HG_ODR_960HZ  = 0x04,
    HG_ODR_1920HZ = 0x05,
    HG_ODR_3840HZ = 0x06,
    HG_ODR_7680HZ = 0x07
};

enum class LSM6DSV80X_XL_Mode : uint8_t
{
    HIGH_PERFORMANCE    = 0x00,
    HIGH_ACCURACY_ODR   = 0x01,
    ODR_TRIGGERED       = 0x03,
    LOW_POWER_2_AVG     = 0x04,
    LOW_POWER_4_AVG     = 0x05,
    LOW_POWER_8_AVG     = 0x06,
    NORMAL              = 0x07
};

enum class LSM6DSV80X_GY_Mode : uint8_t
{
    HIGH_PERFORMANCE    = 0x00,
    HIGH_ACCURACY_ODR   = 0x01,
    ODR_TRIGGERED       = 0x03,
    SLEEP               = 0x04,
    LOW_POWER           = 0x05
};

// HG accelerometer operating mode — CTRL2_XL_HG (0x4E) bits[6:4]
enum class LSM6DSV80X_HG_Mode : uint8_t
{
    HIGH_PERFORMANCE    = 0x00,
    HIGH_ACCURACY_ODR   = 0x01,
    ODR_TRIGGERED       = 0x03
};

enum class LSM6DSV80X_FIFO_Mode : uint8_t
{
    BYPASS              = 0x00,
    FIFO                = 0x01,
    STREAM_WTM_TO_FULL  = 0x02,
    STREAM_TO_FIFO      = 0x03,
    BYPASS_TO_STREAM    = 0x04,
    STREAM              = 0x06,
    BYPASS_TO_FIFO      = 0x07
};

enum class LSM6DSV80X_Status : uint8_t
{
    OK = 0,
    SKIPPED,        // period not elapsed — no new data, not an error
    ERR_COMM,
    ERR_INVALID_ID,
    ERR_CONFIG,
    ERR_TIMEOUT
};

// ─── Raw Data Structs ─────────────────────────────────────────────────────────
// Single base struct — aliased for type clarity at call sites

struct LSM6DSV80X_Raw
{
    int16_t x;
    int16_t y;
    int16_t z;
};

using LSM6DSV80X_RawXL = LSM6DSV80X_Raw;
using LSM6DSV80X_RawGY = LSM6DSV80X_Raw;
using LSM6DSV80X_RawHG = LSM6DSV80X_Raw;

// ─── Other Data Structs ───────────────────────────────────────────────────────

struct LSM6DSV80X_DataReady
{
    bool xl;
    bool gy;
    bool temp;
    bool hg_xl;
};

struct LSM6DSV80X_FifoSample
{
    uint8_t tag;        // Identifies sample type (accel / gyro / timestamp / etc.)
    uint8_t cnt;
    uint8_t data[6];
};

struct LSM6DSV80X_Config
{
    // Low-G accelerometer (disabled by default — set xl_odr = ODR_OFF)
    LSM6DSV80X_XL_Range     xl_range;
    LSM6DSV80X_ODR          xl_odr;
    LSM6DSV80X_XL_Mode      xl_mode;

    // Gyroscope
    LSM6DSV80X_GY_Range     gy_range;
    LSM6DSV80X_ODR          gy_odr;
    LSM6DSV80X_GY_Mode      gy_mode;

    // High-G accelerometer
    bool                    enable_hg_xl;
    LSM6DSV80X_HG_Range     hg_range;
    LSM6DSV80X_HG_ODR       hg_odr;        // Minimum usable: HG_ODR_480HZ
    LSM6DSV80X_HG_Mode      hg_mode;       // Operating mode — CTRL2_XL_HG bits[6:4]

    // Common
    bool                    enable_bdu;         // Block Data Update — recommended ON
    bool                    enable_timestamp;
    bool                    enable_drdy_int1;
    bool                    enable_fifo;
    LSM6DSV80X_FIFO_Mode    fifo_mode;
    uint8_t                 fifo_watermark;     // 0–255 samples
};

// ─── Driver Class ─────────────────────────────────────────────────────────────

class LSM6DSV80X
{
public:
    LSM6DSV80X(i2c_inst_t* i2c, uint8_t address);

    // Lifecycle
    LSM6DSV80X_Status   init(const LSM6DSV80X_Config& config);
    LSM6DSV80X_Status   reset();
    LSM6DSV80X_Status   checkID();

    // Low-G Accelerometer
    LSM6DSV80X_Status   setXLRange(LSM6DSV80X_XL_Range range);
    LSM6DSV80X_Status   getXLRange(LSM6DSV80X_XL_Range& range);
    LSM6DSV80X_Status   setXLODR(LSM6DSV80X_ODR odr);
    LSM6DSV80X_Status   getXLODR(LSM6DSV80X_ODR& odr);
    LSM6DSV80X_Status   setXLMode(LSM6DSV80X_XL_Mode mode);
    LSM6DSV80X_Status   getXLMode(LSM6DSV80X_XL_Mode& mode);

    // Gyroscope
    LSM6DSV80X_Status   setGYRange(LSM6DSV80X_GY_Range range);
    LSM6DSV80X_Status   getGYRange(LSM6DSV80X_GY_Range& range);
    LSM6DSV80X_Status   setGYODR(LSM6DSV80X_ODR odr);
    LSM6DSV80X_Status   getGYODR(LSM6DSV80X_ODR& odr);
    LSM6DSV80X_Status   setGYMode(LSM6DSV80X_GY_Mode mode);
    LSM6DSV80X_Status   getGYMode(LSM6DSV80X_GY_Mode& mode);

    // High-G Accelerometer
    LSM6DSV80X_Status   setHGRange(LSM6DSV80X_HG_Range range);
    LSM6DSV80X_Status   getHGRange(LSM6DSV80X_HG_Range& range);
    LSM6DSV80X_Status   setHGODR(LSM6DSV80X_HG_ODR odr, bool reg_out_en = true);
    LSM6DSV80X_Status   getHGODR(LSM6DSV80X_HG_ODR& odr, bool& reg_out_en);
    LSM6DSV80X_Status   setHGMode(LSM6DSV80X_HG_Mode mode);
    LSM6DSV80X_Status   getHGMode(LSM6DSV80X_HG_Mode& mode);
    // Data Reads
    LSM6DSV80X_Status   readRawXL(LSM6DSV80X_RawXL& raw);
    LSM6DSV80X_Status   readRawGY(LSM6DSV80X_RawGY& raw);
    LSM6DSV80X_Status   readRawHG(LSM6DSV80X_RawHG& raw);
    LSM6DSV80X_Status   readTemperature(int16_t& raw_temp);
    LSM6DSV80X_Status   readTimestamp(uint32_t& timestamp);

    // FIFO
    LSM6DSV80X_Status   setFIFOMode(LSM6DSV80X_FIFO_Mode mode);
    LSM6DSV80X_Status   setFIFOWatermark(uint8_t level);
    uint16_t            readFIFOLevel();
    LSM6DSV80X_Status   readFIFOSample(LSM6DSV80X_FifoSample& sample);
    uint16_t            readFIFO(LSM6DSV80X_FifoSample* buffer, uint16_t max_samples);

    // Status / Interrupts
    LSM6DSV80X_DataReady dataReady();
    bool                fifoWatermark();
    bool                fifoFull();
    bool                fifoOverrun();

    // Self-Test
    LSM6DSV80X_Status   selfTestXL();
    LSM6DSV80X_Status   selfTestGY();

    // Debug — prints key HG register values over serial
    void                debugRegisters();

private:
    i2c_inst_t* _i2c;
    uint8_t     _address;

    bool writeRegister(LSM6DSV80X_Register reg, uint8_t value);
    bool readRegister(LSM6DSV80X_Register reg, uint8_t& value);
    bool readMulti(LSM6DSV80X_Register reg, uint8_t* buffer, uint16_t length);
    bool modifyRegister(LSM6DSV80X_Register reg, uint8_t mask, uint8_t value);

    static int16_t combine(uint8_t low, uint8_t high);
};

#endif // LSM6DSV80X_HPP