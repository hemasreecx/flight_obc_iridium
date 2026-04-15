#include "sensor_manager.hpp"
#include "pin_config.hpp"
#include "config.hpp"
#include "log_format.hpp"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "drivers/kx134/kx134.hpp"
#include "modules/imu_acquisition.hpp"
#include "modules/imu_conversion.hpp"
#include "logging/kx_data_logger.hpp"


#include "drivers/qmc5883l/qmc5883l.hpp"
#include "modules/mag_acquisition.hpp"
#include "modules/mag_conversion.hpp"
#include "logging/qmc_data_logger.hpp"


#if SYSTEM_DEBUG
#define DBG(...) do { printf(__VA_ARGS__); fflush(stdout); } while(0)
#else
#define DBG(...)


static constexpr uint8_t  SENSOR_INIT_RETRIES   = 3;
static constexpr uint32_t SENSOR_RETRY_DELAY_MS = 20000; // 2 sec

namespace sensor_manager
{

static bool _imu_healthy       = false;
static bool _mag_healthy       = false;
static bool _lsm_healthy       = false;
static bool _force_imu_recalib = false;
static bool _force_mag_recalib = false;
static bool _force_lsm_recalib =  false;

static uint32_t _imu_comm_errors       = 0;
static uint32_t _mag_comm_errors       = 0;
static uint32_t _lsm_comm_errors       = 0;
static uint32_t _imu_recovery_attempts = 0;
static uint32_t _mag_recovery_attempts = 0;
static uint32_t _lsm_recovery_attempts = 0;

static KX134          _imu(KX134_I2C_BUS, KX134_I2C_ADDR);
static IMUAcquisition _imu_acq(_imu);

static const KX134_Config KX134_CFG = {
    .range                       = KX134_Range::RANGE_64G,
    .odr                         = KX134_ODR::KX134_ODR_50HZ,
    .performance_mode            = KX134_Performance::HIGH_PERFORMANCE,
    .enable_data_ready_interrupt = false,
    .enable_fifo                 = false
};

static QMC5883L       _mag(MAG_I2C_BUS);
static MagAcquisition _mag_acq(_mag);

static const QMC5883L_Config MAG_CFG = {
    .range                 = QMC5883L_Range::RANGE_8G,
    .odr                   = QMC5883L_ODR::ODR_50HZ,
    .mode                  = QMC5883L_Mode::CONTINUOUS,
    .osr                   = QMC5883L_OSR::OSR_512,
    .enable_drdy_interrupt = false,
    .pointer_roll_over     = true
};
// need to be done for all the i2c buses 
static void i2c_bus_recovery(uint8_t sda_pin, uint8_t scl_pin)
{
    DBG("[sensor] I2C recovery SDA=%d SCL=%d\n", sda_pin, scl_pin);
    gpio_set_function(sda_pin, GPIO_FUNC_SIO);
    gpio_set_function(scl_pin, GPIO_FUNC_SIO);
    gpio_set_dir(sda_pin, GPIO_OUT);
    gpio_set_dir(scl_pin, GPIO_OUT);
    for (int i = 0; i < 9; i++)
    {
        gpio_put(scl_pin, 0); sleep_us(5);
        gpio_put(scl_pin, 1); sleep_us(5);
    }
    gpio_put(sda_pin, 1);
    gpio_put(scl_pin, 1);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    DBG("[sensor] I2C recovery done\n");
}
#endif

static void run_imu_calibration(int16_t& ox, int16_t& oy, int16_t& oz)
{
    DBG("[sensor] IMU calibration — keep still\n");
    gpio_put(LED_PIN, 1);

    int32_t sx = 0, sy = 0, sz = 0;
    int32_t valid = 0;

    for (int i = 0; i < KX134_CALIB_SAMPLES; i++)
    {
        KX134_Raw raw;
        if (_imu.readRaw(raw) == KX134_Status::OK)
        {
            sx += raw.x; sy += raw.y; sz += raw.z;
            valid++;
        }
        sleep_ms(KX134_CALIB_DELAY_MS);
    }

    gpio_put(LED_PIN, 0);

    if (valid == 0)
    {
        DBG("[sensor] IMU calib failed — no valid samples\n");
        ox = oy = oz = 0;
        return;
    }

    float ax    = (float)sx / valid;
    float ay    = (float)sy / valid;
    float az    = (float)sz / valid;
    float one_g = 1.0f / _imu_acq.getConverterScale();
    float abx   = ax < 0.0f ? -ax : ax;
    float aby   = ay < 0.0f ? -ay : ay;
    float abz   = az < 0.0f ? -az : az;

    float ex = (abx > aby && abx > abz) ? one_g : 0.0f;
    float ey = (aby > abx && aby > abz) ? one_g : 0.0f;
    float ez = (abz > abx && abz > aby) ? one_g : 0.0f;

    if (ax < 0.0f) ex = -ex;
    if (ay < 0.0f) ey = -ey;
    if (az < 0.0f) ez = -ez;

    ox = (int16_t)(ax - ex);
    oy = (int16_t)(ay - ey);
    oz = (int16_t)(az - ez);

    _imu_acq.setCalibrationOffsets(ox, oy, oz);
    CalibStore::save(ox, oy, oz);

    DBG("[sensor] IMU calib done X:%d Y:%d Z:%d\n", ox, oy, oz);
}


static void run_mag_calibration(int16_t& ox, int16_t& oy, int16_t& oz)
{
    DBG("[sensor] MAG calibration — rotate in figure-8\n");
    gpio_put(LED_PIN, 1);

    int16_t mnx =  32767, mny =  32767, mnz =  32767;
    int16_t mxx = -32768, mxy = -32768, mxz = -32768;
    int32_t valid = 0;

    for (int i = 0; i < MAG_CALIB_SAMPLES; i++)
    {
        QMC5883L_Raw raw;
        if (_mag.readRaw(raw))
        {
            if (raw.x < mnx) mnx = raw.x;
            if (raw.y < mny) mny = raw.y;
            if (raw.z < mnz) mnz = raw.z;
            if (raw.x > mxx) mxx = raw.x;
            if (raw.y > mxy) mxy = raw.y;
            if (raw.z > mxz) mxz = raw.z;
            valid++;
        }
        sleep_ms(MAG_CALIB_DELAY_MS);
    }

    gpio_put(LED_PIN, 0);

    if (valid == 0)
    {
        DBG("[sensor] MAG calib failed — no valid samples\n");
        ox = oy = oz = 0;
        return;
    }

    ox = (int16_t)((mxx + mnx) / 2);
    oy = (int16_t)((mxy + mny) / 2);
    oz = (int16_t)((mxz + mnz) / 2);

    _mag_acq.setHardIronOffsets(ox, oy, oz);
    float identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
    MagCalibStore::save(ox, oy, oz, identity);

    DBG("[sensor] MAG calib done X:%d Y:%d Z:%d\n", ox, oy, oz);
}


// ============================================================
// KX134 init with retry
// ============================================================


static bool init_kx134_with_retry()
{
    for (uint8_t attempt = 1; attempt <= SENSOR_INIT_RETRIES; attempt++)
    {
        DBG("[sensor] KX134 init attempt %d/%d\n", attempt, SENSOR_INIT_RETRIES);
        _imu.reset();
        sleep_ms(50);

        if (_imu.checkID() != KX134_Status::OK)
        {
            DBG("[sensor] KX134 ID check failed\n");
            i2c_bus_recovery(KX134_SDA_PIN, KX134_SCL_PIN);
            sleep_ms(SENSOR_RETRY_DELAY_MS);
            continue;
        }

        if (_imu.selfTest() != KX134_Status::OK)
            DBG("[sensor] KX134 self-test WARN\n");

        _imu.TrimValues();

        if (_imu.init(KX134_CFG) != KX134_Status::OK)
        {
            DBG("[sensor] KX134 init failed\n");
            sleep_ms(SENSOR_RETRY_DELAY_MS);
            continue;
        }

        _imu_acq.setRateHz(SAMPLE_RATE_HZ);

        int16_t ox = 0, oy = 0, oz = 0;
        CalibData stored;
        if (!_force_imu_recalib && CalibStore::load(stored))
        {
            ox = stored.off_x; oy = stored.off_y; oz = stored.off_z;
            _imu_acq.setCalibrationOffsets(ox, oy, oz);
            DBG("[sensor] IMU calib loaded X:%d Y:%d Z:%d\n", ox, oy, oz);
        }
        else
        {
            run_imu_calibration(ox, oy, oz);
        }

        _imu_acq.resetFilter();
        DBG("[sensor] KX134 OK (attempt %d)\n", attempt);
        return true;
    }

    DBG("[sensor] KX134 ISOLATED after %d attempts\n", SENSOR_INIT_RETRIES);
    return false;
}


// ============================================================
// QMC5883L init with retry
// ============================================================

static bool init_qmc_with_retry()
{
    for (uint8_t attempt = 1; attempt <= SENSOR_INIT_RETRIES; attempt++)
    {
        DBG("[sensor] QMC5883L init attempt %d/%d\n", attempt, SENSOR_INIT_RETRIES);

        if (!_mag.soft_reset())
        {
            DBG("[sensor] QMC soft reset failed\n");
            i2c_bus_recovery(MAG_SDA_PIN, MAG_SCL_PIN);
            sleep_ms(SENSOR_RETRY_DELAY_MS);
            continue;
        }
        sleep_ms(10);

        if (!_mag_acq.init(MAG_CFG))
        {
            DBG("[sensor] QMC acq init failed\n");
            sleep_ms(SENSOR_RETRY_DELAY_MS);
            continue;
        }

        _mag_acq.setTempOffset(32.76f);

        int16_t ox = 0, oy = 0, oz = 0;
        MagCalibData stored;
        if (!_force_mag_recalib && MagCalibStore::load(stored))
        {
            ox = stored.hard_x; oy = stored.hard_y; oz = stored.hard_z;
            _mag_acq.setHardIronOffsets(ox, oy, oz);
            _mag_acq.setSoftIronMatrix(stored.soft_iron);
            DBG("[sensor] MAG calib loaded X:%d Y:%d Z:%d\n", ox, oy, oz);
        }
        else
        {
            run_mag_calibration(ox, oy, oz);
        }

        _mag_acq.resetFilter();
        DBG("[sensor] QMC5883L OK (attempt %d)\n", attempt);
        return true;
    }

    DBG("[sensor] QMC5883L ISOLATED after %d attempts\n", SENSOR_INIT_RETRIES);
    return false;
}

// ============================================================
// init()
// ============================================================

bool init()
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    bool all_ok = true;

    i2c_init(KX134_I2C_BUS, I2C_SPEED_HZ);
    gpio_set_function(KX134_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(KX134_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(KX134_SDA_PIN);
    gpio_pull_up(KX134_SCL_PIN);
    sleep_ms(50);

    _imu_healthy = init_kx134_with_retry();
    if (!_imu_healthy)
    {
        DBG("[sensor] KX134 FAIL — acc fields will be 0\n");
        all_ok = false;
    }

    i2c_init(MAG_I2C_BUS, I2C_SPEED_HZ);
    gpio_set_function(MAG_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MAG_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MAG_SDA_PIN);
    gpio_pull_up(MAG_SCL_PIN);
    sleep_ms(50);

    _mag_healthy = init_qmc_with_retry();
    if (!_mag_healthy)
    {
        DBG("[sensor] QMC FAIL — mag fields will be 0\n");
        all_ok = false;
    }
#endif

    return all_ok;
}

// ============================================================
// task()
// ============================================================

void task()
{

    if (_imu_healthy)
    {
        KX134_Status st = _imu_acq.task();

        if (st == KX134_Status::OK)
        {
            _imu_comm_errors       = 0;
            _imu_recovery_attempts = 0;
        }
        else
        {
            _imu_comm_errors++;
            DBG("[sensor] IMU comm error #%lu\n", _imu_comm_errors);

            if (_imu_comm_errors >= MAX_COMM_ERRORS)
            {
                DBG("[sensor] IMU threshold — recovery\n");
                i2c_bus_recovery(KX134_SDA_PIN, KX134_SCL_PIN);
                _imu.reset();
                sleep_ms(50);

                if (_imu.init(KX134_CFG) == KX134_Status::OK)
                {
                    _imu_recovery_attempts = 0;
                    _imu_comm_errors       = 0;
                    DBG("[sensor] IMU recovery OK\n");
                }
                else
                {
                    _imu_recovery_attempts++;
                    _imu_comm_errors = 0;

                    if (_imu_recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                    {
                        _imu_healthy = false;
                        DBG("[sensor] IMU UNHEALTHY — acc now 0\n");
                    }
                }
            }
        }
    }


    if (_mag_healthy)
    {
        QMC5883L_Status st = _mag_acq.task();

        if (st == QMC5883L_Status::OK ||
            st == QMC5883L_Status::NOT_READY)
        {
            _mag_comm_errors       = 0;
            _mag_recovery_attempts = 0;
        }
        else
        {
            _mag_comm_errors++;
            DBG("[sensor] MAG comm error #%lu\n", _mag_comm_errors);

            if (_mag_comm_errors >= MAX_COMM_ERRORS)
            {
                DBG("[sensor] MAG threshold — recovery\n");
                i2c_bus_recovery(MAG_SDA_PIN, MAG_SCL_PIN);
                _mag.soft_reset();
                sleep_ms(50);

                if (_mag_acq.init(MAG_CFG))
                {
                    _mag_recovery_attempts = 0;
                    _mag_comm_errors       = 0;
                    DBG("[sensor] MAG recovery OK\n");
                }
                else
                {
                    _mag_recovery_attempts++;
                    _mag_comm_errors = 0;

                    if (_mag_recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                    {
                        _mag_healthy = false;
                        DBG("[sensor] MAG UNHEALTHY — mag now 0\n");
                    }
                }
            }
        }
    }

}

// ============================================================
// fill_record()
// Real values when healthy, 0s when disabled or unhealthy
// 
// ============================================================

void fill_record(log_format::Record& r, uint32_t counter)
{
    log_format::init_record(r);
    r.counter = counter;

    if (_imu_healthy)
    {
        KX134_Raw raw;
        if (_imu.readRaw(raw) == KX134_Status::OK)
        {
            r.acc3_x = raw.x;
            r.acc3_y = raw.y;
            r.acc3_z = raw.z;
        }
        // readRaw fail → stays 0
    }

    if (_mag_healthy)
    {
        QMC5883L_Raw raw;
        if (_mag.readRaw(raw))
        {
            r.mag_x = raw.x;   // raw counts — no conversion
            r.mag_y = raw.y;
            r.mag_z = raw.z;
        }
        // readRaw fail → stays 0

        // Temperature still from acquisition (only source)
        const MagSample& s = _mag_acq.getLatestSample();
        r.obc_temperature = (int16_t)(s.temperature * 100.0f);
    }


    // ── GPS ───────────────────────────────────────────────────
#if ENABLE_GPS
    // TODO: real GPS
    r.gps_time  = 0;
    r.latitude  = 0;
    r.longitude = 0;
    r.altitude  = 0;
#else
    // No GPS — use boot timer as timestamp, coords stay 0
    r.gps_time  = to_ms_since_boot(get_absolute_time()) / 1000;
    r.latitude  = 0;
    r.longitude = 0;
    r.altitude  = 0;
#endif

    // ── Thermocouples ─────────────────────────────────────────
#if ENABLE_THERMOCOUPLE
    // TODO: real ADC
#endif
    // disabled → stays 0

    // ── IMU2 ──────────────────────────────────────────────────
#if ENABLE_IMU2
    // TODO: real IMU2 // we have need to fill the code
#endif
    // disabled → stays 0

    // ── Battery ───────────────────────────────────────────────
#if ENABLE_BATTERY
   // we have real ina - need to get that 
#endif
    // disabled → stays 0

    // ── Always last ───────────────────────────────────────────
    r.commit = log_format::COMMIT_COMPLETE;
}

bool imu_healthy() { return _imu_healthy; }
bool mag_healthy() { return _mag_healthy; }
bool lsm_healthy() {return _lsm_healthy; }
// is there anyway i can get the same for ina as welll?
void request_imu_recalib() { _force_imu_recalib = true; }
void request_mag_recalib() { _force_mag_recalib = true; }
bool request_lsm_recalib() {_force_lsm_recalib =  true;  }
} // namespace sensor_manager