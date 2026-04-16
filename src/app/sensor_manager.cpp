#include "sensor_manager.hpp"
#include "pin_config.hpp"
#include "config.hpp"
#include "log_format.hpp"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include <stdio.h>
#include <stdint.h>

#include "logging/ls_data_logger.hpp"

#if SYSTEM_DEBUG
#define DBG(...) do { printf(__VA_ARGS__); fflush(stdout); } while(0)
#else
#define DBG(...)
#endif

static constexpr uint8_t  SENSOR_INIT_RETRIES   = 3;
static constexpr uint32_t SENSOR_RETRY_DELAY_MS = 2000;
static constexpr uint32_t LED_PULSE_ON_MS       = 120;
static constexpr uint32_t LED_PULSE_OFF_MS      = 120;

namespace sensor_manager
{

static bool _imu_healthy       = false;
static bool _mag_healthy       = false;
static bool _lsm_healthy       = false;
static bool _ina_healthy       = false;
static bool _lsm_has_sample    = false;
static bool _force_imu_recalib = false;
static bool _force_mag_recalib = false;
static bool _force_lsm_recalib = false;

static uint32_t _imu_comm_errors       = 0;
static uint32_t _mag_comm_errors       = 0;
static uint32_t _ina_comm_errors       = 0;
static uint32_t _lsm_comm_errors       = 0;
static uint32_t _imu_recovery_attempts = 0;
static uint32_t _mag_recovery_attempts = 0;
static uint32_t _ina_recovery_attempts = 0;
static uint32_t _lsm_recovery_attempts = 0;

static DataLogger _kx_logger(LoggerMode::CONVERTED);
#if SENSOR_LOG_RAW
static MagDataLogger _qmc_logger(MagLoggerMode::PRE_CALIB);
#else
static MagDataLogger _qmc_logger(MagLoggerMode::CONVERTED);
#endif

static KX134          _imu(KX134_I2C_BUS, KX134_I2C_ADDR_LOW);
static IMUAcquisition _imu_acq(_imu, _kx_logger);

static const KX134_Config KX134_CFG = {
    .range                       = KX134_Range::RANGE_64G,
    .odr                         = KX134_ODR::KX134_ODR_50HZ,
    .performance_mode            = KX134_Performance::HIGH_PERFORMANCE,
    .enable_data_ready_interrupt = false,
    .enable_fifo                 = false
};

static QMC5883L       _mag(MAG_I2C_BUS);
static MagAcquisition _mag_acq(_mag, _qmc_logger);
static LSM6DSV80X     _lsm(LS_I2C_BUS, LSM6DSV80X_I2C_ADDR_LOW);
static LSM6DSV80X_Acquisition _lsm_acq(_lsm, SAMPLE_RATE_HZ);
static Ina260         _ina(INA260_I2C_BUS);

static float _ina_current_ma = 0.0f;
static float _ina_voltage_mv = 0.0f;
static bool  _ina_has_sample = false;

static const QMC5883L_Config MAG_CFG = {
    .range                 = QMC5883L_Range::RANGE_8G,
    .odr                   = QMC5883L_ODR::ODR_50HZ,
    .mode                  = QMC5883L_Mode::CONTINUOUS,
    .osr                   = QMC5883L_OSR::OSR_512,
    .enable_drdy_interrupt = false,
    .pointer_roll_over     = true
};

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

static void ina_set_enabled(bool enabled)
{
    gpio_put(INA260_EN, enabled ? 1 : 0);
}

static void ina_power_cycle()
{
    DBG("[sensor] INA power cycle via EN pin\n");
    ina_set_enabled(false);
    sleep_ms(20);
    ina_set_enabled(true);
    sleep_ms(20);
}

// LED pulse code for calibration visibility:
// 1 pulse = KX134 IMU recalibration
// 2 pulses = QMC magnetometer recalibration
// 3 pulses = LSM IMU recalibration/defaulting requested
static void led_pulse_code(uint8_t pulses)
{
    for (uint8_t i = 0; i < pulses; i++)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(LED_PULSE_ON_MS);
        gpio_put(LED_PIN, 0);
        sleep_ms(LED_PULSE_OFF_MS);
    }
    sleep_ms(200);
}

static int16_t to_i16_saturated(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;

    // Round to nearest integer instead of truncating.
    if (value >= 0.0f)
        return static_cast<int16_t>(value + 0.5f);
    return static_cast<int16_t>(value - 0.5f);
}

static uint16_t to_u16_saturated(float value)
{
    if (value > 65535.0f) return 65535;
    if (value < 0.0f) return 0;

    // Round to nearest integer instead of truncating.
    return static_cast<uint16_t>(value + 0.5f);
}

static float lsm_raw_temp_to_celsius(int16_t raw_temp)
{
    return (static_cast<float>(raw_temp) / 256.0f) + 25.0f;
}

static void apply_lsm_calibration_from_store_or_default()
{
    LSM6DSV80X_CalibData stored{};
    if (!_force_lsm_recalib && LSM6DSV80X_CalibStore::load(stored))
    {
        _lsm_acq.setGYOffsets(stored.gy_off_x, stored.gy_off_y, stored.gy_off_z);
        _lsm_acq.setHGOffsets(stored.hg_off_x, stored.hg_off_y, stored.hg_off_z);
        _lsm_acq.setTempOffset(stored.temp_off);
        DBG("[sensor] LSM calib loaded\n");
    }
    else
    {
        if (_force_lsm_recalib)
            led_pulse_code(3);

        _lsm_acq.clearGYOffsets();
        _lsm_acq.clearHGOffsets();
        _lsm_acq.clearTempOffset();
        DBG("[sensor] LSM calib defaulted (zeros)\n");
    }

    _lsm_acq.resetFilters();
}

static void apply_imu_calibration_from_store_or_zero()
{
    int16_t ox = 0, oy = 0, oz = 0;
    CalibData stored{};
    if (CalibStore::load(stored))
    {
        ox = stored.off_x; oy = stored.off_y; oz = stored.off_z;
        DBG("[sensor] IMU calib (recovery) loaded X:%d Y:%d Z:%d\n", ox, oy, oz);
    }
    else
    {
        DBG("[sensor] IMU calib (recovery) defaulted to 0,0,0\n");
    }
    _imu_acq.setCalibrationOffsets(ox, oy, oz);
}

static void apply_mag_calibration_from_store_or_default()
{
    int16_t ox = 0, oy = 0, oz = 0;
    MagCalibData stored{};
    if (!_force_mag_recalib && MagCalibStore::load(stored))
    {
        ox = stored.hard_x; oy = stored.hard_y; oz = stored.hard_z;
        _mag_acq.setHardIronOffsets(ox, oy, oz);
        _mag_acq.setSoftIronMatrix(stored.soft_iron);
        DBG("[sensor] MAG calib loaded X:%d Y:%d Z:%d\n", ox, oy, oz);
    }
    else
    {
        _mag_acq.setHardIronOffsets(0, 0, 0);
        float identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
        _mag_acq.setSoftIronMatrix(identity);
        DBG("[sensor] MAG calib defaulted (zeros)\n");
    }
    _mag_acq.resetFilter();
}

static void run_imu_calibration(int16_t& ox, int16_t& oy, int16_t& oz)
{
    DBG("[sensor] IMU calibration — keep still\n");
    led_pulse_code(1);
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

    ox = to_i16_saturated(ax - ex);
    oy = to_i16_saturated(ay - ey);
    oz = to_i16_saturated(az - ez);

    _imu_acq.setCalibrationOffsets(ox, oy, oz);
    CalibStore::save(ox, oy, oz);

    DBG("[sensor] IMU calib done X:%d Y:%d Z:%d\n", ox, oy, oz);
}


static void run_mag_calibration(int16_t& ox, int16_t& oy, int16_t& oz)
{
    DBG("[sensor] MAG calibration — rotate in figure-8\n");
    led_pulse_code(2);
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
    _mag_acq.setSoftIronMatrix(identity);
    MagCalibStore::save(ox, oy, oz, identity);

    DBG("[sensor] MAG calib done X:%d Y:%d Z:%d\n", ox, oy, oz);
}

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


static bool init_lsm_with_retry()
{
    for (uint8_t attempt = 1; attempt <= SENSOR_INIT_RETRIES; attempt++)
    {
        DBG("[sensor] LSM6DSV80X init attempt %d/%d\n", attempt, SENSOR_INIT_RETRIES);

        if (_lsm.checkID() != LSM6DSV80X_Status::OK)
        {
            DBG("[sensor] LSM ID check failed\n");
            i2c_bus_recovery(LS_SDA_PIN, LS_SCL_PIN);
            sleep_ms(SENSOR_RETRY_DELAY_MS);
            continue;
        }

        if (!_lsm_acq.init())
        {
            DBG("[sensor] LSM acquisition init failed\n");
            sleep_ms(SENSOR_RETRY_DELAY_MS);
            continue;
        }

        apply_lsm_calibration_from_store_or_default();
        _lsm_has_sample = false;
        _lsm_comm_errors = 0;
        _lsm_recovery_attempts = 0;
        DBG("[sensor] LSM6DSV80X OK (attempt %d)\n", attempt);
        return true;
    }

    DBG("[sensor] LSM6DSV80X ISOLATED after %d attempts\n", SENSOR_INIT_RETRIES);
    return false;
}


static bool init_ina_with_retry()
{
    for (uint8_t attempt = 1; attempt <= SENSOR_INIT_RETRIES; attempt++)
    {
        DBG("[sensor] INA260 init attempt %d/%d\n", attempt, SENSOR_INIT_RETRIES);

        if (attempt > 1)
            ina_power_cycle();

        if (!_ina.reset())
        {
            DBG("[sensor] INA reset failed\n");
            i2c_bus_recovery(INA260_SDA_PIN, INA260_SCL_PIN);
            sleep_ms(SENSOR_RETRY_DELAY_MS);
            continue;
        }

        sleep_ms(10);

        if (!_ina.init())
        {
            DBG("[sensor] INA init failed\n");
            sleep_ms(SENSOR_RETRY_DELAY_MS);
            continue;
        }

        _ina_comm_errors = 0;
        _ina_recovery_attempts = 0;
        _ina_has_sample = false;
        DBG("[sensor] INA260 OK (attempt %d)\n", attempt);
        return true;
    }

    DBG("[sensor] INA260 ISOLATED after %d attempts\n", SENSOR_INIT_RETRIES);
    return false;
}


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

        _mag_acq.setTempOffset(MAG_TEMP_OFFSET_C);

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

    // INA260 power-enable pin (active high): ensure sensor rail is ON before INA init
    gpio_init(INA260_EN);
    gpio_set_dir(INA260_EN, GPIO_OUT);
    ina_set_enabled(true);
    sleep_ms(10);

    bool all_ok = true;

    if (_force_imu_recalib) led_pulse_code(1);
    if (_force_mag_recalib) led_pulse_code(2);
    if (_force_lsm_recalib) led_pulse_code(3);

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

    i2c_init(LS_I2C_BUS, I2C_SPEED_HZ);
    gpio_set_function(LS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(LS_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(LS_SDA_PIN);
    gpio_pull_up(LS_SCL_PIN);
    sleep_ms(50);

    _lsm_healthy = init_lsm_with_retry();
    if (!_lsm_healthy)
    {
        DBG("[sensor] LSM FAIL — imu2 and obc_temp fields will be 0\n");
        all_ok = false;
    }

    i2c_init(INA260_I2C_BUS, I2C_SPEED_HZ);
    gpio_set_function(INA260_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(INA260_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(INA260_SDA_PIN);
    gpio_pull_up(INA260_SCL_PIN);
    sleep_ms(50);

    _ina_healthy = init_ina_with_retry();
    if (!_ina_healthy)
    {
        DBG("[sensor] INA260 FAIL — battery fields will be 0\n");
        all_ok = false;
    }

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
                    apply_imu_calibration_from_store_or_zero();
                    _imu_acq.resetFilter();
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


    if (_lsm_healthy)
    {
        LSM6DSV80X_Status st = _lsm_acq.task();

        if (st == LSM6DSV80X_Status::OK || st == LSM6DSV80X_Status::SKIPPED)
        {
            if (st == LSM6DSV80X_Status::OK)
                _lsm_has_sample = true;
            _lsm_comm_errors = 0;
            _lsm_recovery_attempts = 0;
        }
        else
        {
            _lsm_comm_errors++;
            DBG("[sensor] LSM comm error #%lu\n", _lsm_comm_errors);

            if (_lsm_comm_errors >= MAX_COMM_ERRORS)
            {
                DBG("[sensor] LSM threshold — recovery\n");
                i2c_bus_recovery(LS_SDA_PIN, LS_SCL_PIN);
                _lsm.reset();
                sleep_ms(50);

                if (_lsm_acq.init())
                {
                    apply_lsm_calibration_from_store_or_default();
                    _lsm_has_sample = false;
                    _lsm_recovery_attempts = 0;
                    _lsm_comm_errors = 0;
                    DBG("[sensor] LSM recovery OK\n");
                }
                else
                {
                    _lsm_recovery_attempts++;
                    _lsm_comm_errors = 0;

                    if (_lsm_recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                    {
                        _lsm_healthy = false;
                        _lsm_has_sample = false;
                        DBG("[sensor] LSM UNHEALTHY — imu2 and obc_temp now 0\n");
                    }
                }
            }
        }
    }


    if (_ina_healthy)
    {
        float current_ma = 0.0f;
        float voltage_mv = 0.0f;

        if (_ina.readAll(current_ma, voltage_mv))
        {
            _ina_current_ma = current_ma;
            _ina_voltage_mv = voltage_mv;
            _ina_has_sample = true;
            _ina_comm_errors = 0;
            _ina_recovery_attempts = 0;
        }
        else
        {
            _ina_comm_errors++;
            DBG("[sensor] INA comm error #%lu\n", _ina_comm_errors);

            if (_ina_comm_errors >= MAX_COMM_ERRORS)
            {
                DBG("[sensor] INA threshold — recovery\n");
                i2c_bus_recovery(INA260_SDA_PIN, INA260_SCL_PIN);
                ina_power_cycle();
                _ina.reset();
                sleep_ms(50);

                if (_ina.init())
                {
                    _ina_recovery_attempts = 0;
                    _ina_comm_errors = 0;
                    _ina_has_sample = false;
                    DBG("[sensor] INA recovery OK\n");
                }
                else
                {
                    _ina_recovery_attempts++;
                    _ina_comm_errors = 0;

                    if (_ina_recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                    {
                        _ina_healthy = false;
                        _ina_has_sample = false;
                        DBG("[sensor] INA UNHEALTHY — battery now 0\n");
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
                    apply_mag_calibration_from_store_or_default();
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
        if (_imu_acq.hasSample())
        {
            const IMUSample& s = _imu_acq.getLatestSample();
#if SENSOR_LOG_RAW
            r.acc3_x = s.raw_x;
            r.acc3_y = s.raw_y;
            r.acc3_z = s.raw_z;
#else
            // Store converted acceleration in milli-g as int16_t.
            r.acc3_x = to_i16_saturated(s.x_g * 1000.0f);
            r.acc3_y = to_i16_saturated(s.y_g * 1000.0f);
            r.acc3_z = to_i16_saturated(s.z_g * 1000.0f);
#endif
        }
    }

    if (_mag_healthy)
    {
        const MagSample& s = _mag_acq.getLatestSample();
#if SENSOR_LOG_RAW
        r.mag_x = s.raw_x;
        r.mag_y = s.raw_y;
        r.mag_z = s.raw_z;
#else
        r.mag_x = to_i16_saturated(s.x_gauss * 1000.0f);
        r.mag_y = to_i16_saturated(s.y_gauss * 1000.0f);
        r.mag_z = to_i16_saturated(s.z_gauss * 1000.0f);
#endif
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
    r.gps_time  = counter;
    r.latitude  = 0;
    r.longitude = 0;
    r.altitude  = 0;
#endif

    // ── Thermocouples ─────────────────────────────────────────
#if ENABLE_THERMOCOUPLE
    // TODO: real ADC
#endif
    // disabled → stays 0

    // ── IMU2 (LSM6DSV80X) ─────────────────────────────────────
    if (_lsm_healthy && _lsm_has_sample)
    {
        const LSM6DSV80X_Data hg = _lsm_acq.getFilteredHG();
        const LSM6DSV80X_Data gy = _lsm_acq.getFilteredGY();

        // HG is in g -> centi-g for int16 storage.
        r.imu_acc_x = to_i16_saturated(hg.x * 100.0f);
        r.imu_acc_y = to_i16_saturated(hg.y * 100.0f);
        r.imu_acc_z = to_i16_saturated(hg.z * 100.0f);

        // GY is in dps -> deci-dps for int16 storage.
        r.imu_gyro_x = to_i16_saturated(gy.x * 10.0f);
        r.imu_gyro_y = to_i16_saturated(gy.y * 10.0f);
        r.imu_gyro_z = to_i16_saturated(gy.z * 10.0f);

        const float lsm_temp_c = lsm_raw_temp_to_celsius(_lsm_acq.getRawTemp());
        r.obc_temperature = to_i16_saturated(lsm_temp_c * 100.0f);
    }

    // ── Battery ───────────────────────────────────────────────
    if (_ina_healthy && _ina_has_sample)
    {
        // INA driver outputs mV and mA.
        // This is equivalent to V*1000 and A*1000 in integer record fields.
        r.battery_voltage = to_u16_saturated(_ina_voltage_mv);
        r.battery_current = to_i16_saturated(_ina_current_ma);
    }
    // INA unavailable/invalid sample -> battery fields stay 0

    // ── Always last ───────────────────────────────────────────
    r.commit = log_format::COMMIT_COMPLETE;
}

bool imu_healthy() { return _imu_healthy; }
bool mag_healthy() { return _mag_healthy; }
bool lsm_healthy() {return _lsm_healthy; }
bool ina_healthy() { return _ina_healthy; }
void request_imu_recalib() { _force_imu_recalib = true; }
void request_mag_recalib() { _force_mag_recalib = true; }
void request_lsm_recalib() { _force_lsm_recalib = true; }
} // namespace sensor_manager