#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdlib.h>   // rand(), srand()
#include <stdint.h>

#include "pin_config.hpp"
#include "log_format.hpp"

// ── Drivers ──────────────────────────────────────────────────
#include "drivers/kx134.hpp"
#include "drivers/qmc5883l.hpp"

// ── Modules ──────────────────────────────────────────────────
#include "modules/imu_conversion.hpp"
#include "modules/imu_acquisition.hpp"
#include "modules/mag_conversion.hpp"
#include "modules/mag_acquisition.hpp"

// ── Logging ──────────────────────────────────────────────────
#include "logging/kx_data_logger.hpp"
#include "logging/qmc_data_logger.hpp"

// ── Iridium ──────────────────────────────────────────────────
#include "rockblock_9603/iridium_driver.hpp"
#include "rockblock_9603/rockblock_manager.hpp"

/* ============================================================
   SYSTEM CONFIGURATION
   ============================================================ */

#define SYSTEM_DEBUG            1
#define LOGGER_RAW_MODE         0

#define I2C_SPEED_HZ            400000
#define KX134_ADDRESS           0x1E
#define SAMPLE_RATE_HZ          50

#define MAX_COMM_ERRORS         10
#define MAX_RECOVERY_ATTEMPTS   3

#define CALIB_SAMPLES           560
#define CALIB_SAMPLE_DELAY_MS   5
#define RECALIB_PROMPT_MS       5000

#define LED_PIN                 25

// Iridium TX interval — RockBlock minimum ~20s recommended
#define TX_INTERVAL_MS          20000

/* ============================================================
   DEBUG MACRO
   ============================================================ */

#if SYSTEM_DEBUG
#define DEBUG_PRINT(...) do { printf(__VA_ARGS__); fflush(stdout); } while(0)
#else
#define DEBUG_PRINT(...)
#endif

/* ============================================================
   LOGGER OBJECTS
   ============================================================ */

#if LOGGER_RAW_MODE
static DataLogger    kx_logger(LoggerMode::RAW);
static MagDataLogger qmc_logger(MagLoggerMode::RAW);
#else
static DataLogger    kx_logger(LoggerMode::CONVERTED);
static MagDataLogger qmc_logger(MagLoggerMode::CONVERTED);
#endif

/* ============================================================
   GLOBAL SENSOR OBJECTS
   ============================================================ */

static KX134         imu(KX134_I2C_BUS,  KX134_ADDRESS);
static QMC5883L      mag(MAG_I2C_BUS);
static IMUAcquisition imu_acq(imu, kx_logger);
static MagAcquisition mag_acq(mag, qmc_logger);

static const QMC5883L_Config MAG_CONFIG = {
    .range                 = QMC5883L_Range::RANGE_8G,
    .odr                   = QMC5883L_ODR::ODR_50HZ,
    .mode                  = QMC5883L_Mode::CONTINUOUS,
    .osr                   = QMC5883L_OSR::OSR_512,
    .enable_drdy_interrupt = false,
    .pointer_roll_over     = true
};

/* ============================================================
   SYSTEM STATE
   ============================================================ */

static volatile bool     system_initialized = false;
static volatile uint32_t loop_count         = 0;
static volatile uint32_t imu_comm_errors    = 0;
static volatile uint32_t mag_comm_errors    = 0;
static volatile uint32_t imu_recovery_attempts = 0;
static volatile uint32_t mag_recovery_attempts = 0;

// Counter for log_format::Record
static uint32_t record_counter = 0;

/* ============================================================
   LED HELPERS
   ============================================================ */

static void led_init()
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
}

static void led_on()  { gpio_put(LED_PIN, 1); }
static void led_off() { gpio_put(LED_PIN, 0); }

/* ============================================================
   RNG HELPERS — for simulated fields
   ============================================================ */

static void rng_init()
{
    srand((unsigned int)to_ms_since_boot(get_absolute_time()));
}

static int rng_range(int lo, int hi)
{
    return lo + rand() % (hi - lo + 1);
}

/* ============================================================
   I2C BUS RECOVERY
   ============================================================ */

static void i2c_bus_recovery(uint8_t sda_pin, uint8_t scl_pin)
{
    DEBUG_PRINT("I2C bus recovery on SDA=%d SCL=%d\n", sda_pin, scl_pin);

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

    DEBUG_PRINT("I2C bus recovery complete\n");
}

/* ============================================================
   IMU RECOVERY
   ============================================================ */

static bool imu_recover()
{
    DEBUG_PRINT("Attempting IMU recovery\n");
    imu.reset();
    sleep_ms(50);

    if (imu.checkID() != KX134_Status::OK)
    {
        DEBUG_PRINT("IMU ID check failed after reset\n");
        return false;
    }
    if (!imu_acq.init())
    {
        DEBUG_PRINT("IMU acquisition re-init failed\n");
        return false;
    }

    DEBUG_PRINT("IMU recovery OK\n");
    return true;
}

/* ============================================================
   MAG RECOVERY
   ============================================================ */

static bool mag_recover()
{
    DEBUG_PRINT("Attempting MAG recovery\n");

    if (!mag.soft_reset())
    {
        DEBUG_PRINT("MAG soft reset failed\n");
        return false;
    }
    sleep_ms(50);

    if (!mag_acq.init(MAG_CONFIG))
    {
        DEBUG_PRINT("MAG acquisition re-init failed\n");
        return false;
    }

    DEBUG_PRINT("MAG recovery OK\n");
    return true;
}

/* ============================================================
   IMU CALIBRATION
   ============================================================ */

static void run_imu_calibration(int16_t& out_x,
                                 int16_t& out_y,
                                 int16_t& out_z)
{
    DEBUG_PRINT("IMU calibration — keep sensor still\n");
    led_on();

    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int32_t valid  = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        KX134_Raw raw;
        if (imu.readRaw(raw) == KX134_Status::OK)
        {
            sum_x += raw.x;
            sum_y += raw.y;
            sum_z += raw.z;
            valid++;
        }
        sleep_ms(CALIB_SAMPLE_DELAY_MS);
    }

    led_off();

    if (valid == 0)
    {
        DEBUG_PRINT("IMU calibration failed — no valid samples\n");
        out_x = out_y = out_z = 0;
        return;
    }

    float avg_x = (float)sum_x / valid;
    float avg_y = (float)sum_y / valid;
    float avg_z = (float)sum_z / valid;

    float one_g  = 1.0f / imu_acq.getConverterScale();
    float abs_x  = avg_x < 0.0f ? -avg_x : avg_x;
    float abs_y  = avg_y < 0.0f ? -avg_y : avg_y;
    float abs_z  = avg_z < 0.0f ? -avg_z : avg_z;

    float exp_x = (abs_x > abs_y && abs_x > abs_z) ? one_g : 0.0f;
    float exp_y = (abs_y > abs_x && abs_y > abs_z) ? one_g : 0.0f;
    float exp_z = (abs_z > abs_x && abs_z > abs_y) ? one_g : 0.0f;

    if (avg_x < 0.0f) exp_x = -exp_x;
    if (avg_y < 0.0f) exp_y = -exp_y;
    if (avg_z < 0.0f) exp_z = -exp_z;

    out_x = (int16_t)(avg_x - exp_x);
    out_y = (int16_t)(avg_y - exp_y);
    out_z = (int16_t)(avg_z - exp_z);

    imu_acq.setCalibrationOffsets(out_x, out_y, out_z);
    CalibStore::save(out_x, out_y, out_z);

    DEBUG_PRINT("IMU calib done — X:%d Y:%d Z:%d\n", out_x, out_y, out_z);
}

/* ============================================================
   MAG CALIBRATION
   ============================================================ */

static void run_mag_calibration(int16_t& out_x,
                                 int16_t& out_y,
                                 int16_t& out_z)
{
    DEBUG_PRINT("MAG calibration — rotate sensor in figure-8\n");
    led_on();

    int16_t min_x =  32767, min_y =  32767, min_z =  32767;
    int16_t max_x = -32768, max_y = -32768, max_z = -32768;
    int32_t valid  = 0;

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        QMC5883L_Raw raw;
        if (mag.readRaw(raw))
        {
            if (raw.x < min_x) min_x = raw.x;
            if (raw.y < min_y) min_y = raw.y;
            if (raw.z < min_z) min_z = raw.z;
            if (raw.x > max_x) max_x = raw.x;
            if (raw.y > max_y) max_y = raw.y;
            if (raw.z > max_z) max_z = raw.z;
            valid++;
            qmc_logger.logRaw(raw.x, raw.y, raw.z);
        }
        sleep_ms(CALIB_SAMPLE_DELAY_MS);
    }

    led_off();

    if (valid == 0)
    {
        DEBUG_PRINT("MAG calibration failed — no valid samples\n");
        out_x = out_y = out_z = 0;
        return;
    }

    out_x = (int16_t)((max_x + min_x) / 2);
    out_y = (int16_t)((max_y + min_y) / 2);
    out_z = (int16_t)((max_z + min_z) / 2);

    mag_acq.setHardIronOffsets(out_x, out_y, out_z);

    float identity[9] = { 1,0,0, 0,1,0, 0,0,1 };
    MagCalibStore::save(out_x, out_y, out_z, identity);

    DEBUG_PRINT("MAG calib done — X:%d Y:%d Z:%d\n", out_x, out_y, out_z);
}

/* ============================================================
   RECALIBRATION PROMPT
   ============================================================ */

static bool wait_for_recalib_request()
{
    printf("Send 'c' within %d seconds to force recalibration...\n",
           RECALIB_PROMPT_MS / 1000);
    fflush(stdout);

    uint32_t deadline = to_ms_since_boot(get_absolute_time())
                        + RECALIB_PROMPT_MS;

    while (to_ms_since_boot(get_absolute_time()) < deadline)
    {
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT && (ch == 'c' || ch == 'C'))
        {
            printf("Recalibration requested.\n");
            fflush(stdout);
            return true;
        }
        sleep_ms(10);
    }
    return false;
}

/* ============================================================
   FILL RECORD
   Real:      acc3_x/y/z (KX134 raw counts)
              mag_x/y/z  (QMC gauss × 1000 → int16)
              imu_temperature (QMC temp × 100 → int16)
              counter

   Simulated: everything else
   ============================================================ */

static void fill_record(log_format::Record& r)
{
    log_format::init_record(r);

    r.counter = record_counter++;

    // ── REAL: KX134 accelerometer ─────────────────────────────
    KX134_Raw    kx_raw;
    IMU_Data     kx_conv;
    // imu_acq.task() logs internally — read raw directly for record
    if (imu.readRaw(kx_raw) == KX134_Status::OK)
    {
        r.acc3_x = kx_raw.x;
        r.acc3_y = kx_raw.y;
        r.acc3_z = kx_raw.z;
    }

    // ── REAL: QMC5883L magnetometer + temperature ─────────────
    const MagSample& mag_sample = mag_acq.getLatestSample();
    r.mag_x = (int16_t)(mag_sample.x_gauss * 1000.0f);
    r.mag_y = (int16_t)(mag_sample.y_gauss * 1000.0f);
    r.mag_z = (int16_t)(mag_sample.z_gauss * 1000.0f);

    // imu_temperature field reused for mag temperature
    // stored as °C × 100 → int16
    // Note: field named imu_temperature but carries mag temp here
    r.imu_temperature = (int16_t)(mag_sample.temperature * 100.0f);

    // ── SIMULATED: GPS ────────────────────────────────────────
    r.gps_time  = to_ms_since_boot(get_absolute_time()) / 1000;
    r.latitude  = rng_range(280000000, 280100000);   // ~28.0°N
    r.longitude = rng_range(770000000, 770100000);   // ~77.0°E
    r.altitude  = rng_range(200000,    250000);      // 200–250m in mm

    // ── SIMULATED: 20 thermocouples (°C × 10) ─────────────────
    for (int i = 0; i < 20; i++)
        r.thermocouples[i] = (int16_t)rng_range(200, 400);

    // ── SIMULATED: IMU (separate from KX134) ──────────────────
    r.imu_acc_x  = (int16_t)rng_range(-500,  500);
    r.imu_acc_y  = (int16_t)rng_range(-500,  500);
    r.imu_acc_z  = (int16_t)rng_range(9500, 10500);
    r.imu_gyro_x = (int16_t)rng_range(-50,    50);
    r.imu_gyro_y = (int16_t)rng_range(-50,    50);
    r.imu_gyro_z = (int16_t)rng_range(-50,    50);

    // ── SIMULATED: Battery ────────────────────────────────────
    r.battery_voltage = (uint16_t)rng_range(7200, 8400);
    r.battery_current = (int16_t) rng_range(300,  600);

    r.commit = log_format::COMMIT_COMPLETE;
}

/* ============================================================
   SYSTEM INIT
   ============================================================ */

static bool system_init(bool force_recalib)
{
    DEBUG_PRINT("\n=== OBC_FLIGHT BOOT ===\n");

    led_init();
    rng_init();

    // ── I2C0 — KX134 ─────────────────────────────────────────
    i2c_init(KX134_I2C_BUS, I2C_SPEED_HZ);
    gpio_set_function(KX134_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(KX134_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(KX134_SDA_PIN);
    gpio_pull_up(KX134_SCL_PIN);
    DEBUG_PRINT("I2C0 (KX134) init OK SDA=%d SCL=%d\n",
                KX134_SDA_PIN, KX134_SCL_PIN);

    // ── I2C1 — QMC5883L ──────────────────────────────────────
    i2c_init(MAG_I2C_BUS, I2C_SPEED_HZ);
    gpio_set_function(MAG_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MAG_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MAG_SDA_PIN);
    gpio_pull_up(MAG_SCL_PIN);
    DEBUG_PRINT("I2C1 (QMC5883L) init OK SDA=%d SCL=%d\n",
                MAG_SDA_PIN, MAG_SCL_PIN);

    sleep_ms(100);

    // ── KX134 init ────────────────────────────────────────────
    if (imu.checkID() != KX134_Status::OK)
    {
        DEBUG_PRINT("ERROR: KX134 not detected\n");
        return false;
    }
    imu.TrimValues();
    if (imu.selfTest() != KX134_Status::OK)
        DEBUG_PRINT("WARNING: KX134 self-test failed\n");
    if (!imu_acq.init())
    {
        DEBUG_PRINT("ERROR: IMU acquisition init failed\n");
        return false;
    }
    imu_acq.setRateHz(SAMPLE_RATE_HZ);
    DEBUG_PRINT("KX134 init OK\n");

    // ── QMC5883L init ─────────────────────────────────────────
    if (!mag_acq.init(MAG_CONFIG))
    {
        DEBUG_PRINT("ERROR: MAG acquisition init failed\n");
        return false;
    }
    mag_acq.setTempOffset(32.76f);
    DEBUG_PRINT("QMC5883L init OK\n");

    // ── IMU Calibration ───────────────────────────────────────
    {
        int16_t ox = 0, oy = 0, oz = 0;
        CalibData stored;
        if (!force_recalib && CalibStore::load(stored))
        {
            ox = stored.off_x;
            oy = stored.off_y;
            oz = stored.off_z;
            imu_acq.setCalibrationOffsets(ox, oy, oz);
            DEBUG_PRINT("IMU calib loaded from flash X:%d Y:%d Z:%d\n",
                        ox, oy, oz);
        }
        else
        {
            run_imu_calibration(ox, oy, oz);
            printf("IMU calibration done. Prepare to rotate sensor in figure-8...\n");
            fflush(stdout);
            sleep_ms(3000);
        }
        imu_acq.resetFilter();
    }

    // ── MAG Calibration ───────────────────────────────────────
    {
        int16_t ox = 0, oy = 0, oz = 0;
        MagCalibData stored;
        if (!force_recalib && MagCalibStore::load(stored))
        {
            ox = stored.hard_x;
            oy = stored.hard_y;
            oz = stored.hard_z;
            mag_acq.setHardIronOffsets(ox, oy, oz);
            mag_acq.setSoftIronMatrix(stored.soft_iron);
            DEBUG_PRINT("MAG calib loaded from flash X:%d Y:%d Z:%d\n",
                        ox, oy, oz);
        }
        else
        {
            run_mag_calibration(ox, oy, oz);
        }
        mag_acq.resetFilter();
    }

    // ── Iridium init ──────────────────────────────────────────
    iridium_driver::Config iridium_cfg;
    iridium_cfg.uart_inst       = IRIDIUM_UART;
    iridium_cfg.tx_pin = IRIDIUM_TX_PIN;  
    iridium_cfg.rx_pin = IRIDIUM_RX_PIN;  
    iridium_cfg.baud_rate  = 19200;
    iridium_cfg.sleep_pin  = 0xFF; //IRIDIUM_ONOFF_PIN;
    iridium_cfg.netavb_pin = 0xFF; //IRIDIUM_NETAVB_PIN;
    iridium_cfg.ri_pin     = 0xFF;//IRIDIUM_RI_PIN;

    if (!rockblock_manager::init(iridium_cfg))
    {
        DEBUG_PRINT("ERROR: Iridium init failed\n");
        return false;
    }
    rockblock_manager::set_tx_interval_ms(TX_INTERVAL_MS);
    DEBUG_PRINT("Iridium init OK\n");

    // ── CSV headers ───────────────────────────────────────────
    kx_logger.printHeader();
    qmc_logger.printHeader();
    fflush(stdout);

    system_initialized = true;
    DEBUG_PRINT("=== SYSTEM INIT COMPLETE ===\n");
    return true;
}

/* ============================================================
   MAIN LOOP
   ============================================================ */

static void main_loop()
{
    // ── IMU task ──────────────────────────────────────────────
    KX134_Status imu_status = imu_acq.task();

    if (imu_status == KX134_Status::OK)
    {
        imu_comm_errors       = 0;
        imu_recovery_attempts = 0;
    }
    else if (imu_status != KX134_Status::OK)
    {
        imu_comm_errors++;
        if (imu_comm_errors >= MAX_COMM_ERRORS)
        {
            i2c_bus_recovery(KX134_SDA_PIN, KX134_SCL_PIN);
            if (!imu_recover())
            {
                imu_recovery_attempts++;
                if (imu_recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                {
                    DEBUG_PRINT("FATAL: IMU unrecoverable. Halting.\n");
                    while (true) { tight_loop_contents(); }
                }
            }
            else { imu_recovery_attempts = 0; }
            imu_comm_errors = 0;
        }
    }

    // ── MAG task ──────────────────────────────────────────────
    QMC5883L_Status mag_status = mag_acq.task();

    if (mag_status == QMC5883L_Status::OK)
    {
        mag_comm_errors       = 0;
        mag_recovery_attempts = 0;
    }
    else if (mag_status == QMC5883L_Status::READ_ERROR ||
             mag_status == QMC5883L_Status::ERR_COMM)
    {
        mag_comm_errors++;
        if (mag_comm_errors >= MAX_COMM_ERRORS)
        {
            i2c_bus_recovery(MAG_SDA_PIN, MAG_SCL_PIN);
            if (!mag_recover())
            {
                mag_recovery_attempts++;
                if (mag_recovery_attempts >= MAX_RECOVERY_ATTEMPTS)
                {
                    DEBUG_PRINT("FATAL: MAG unrecoverable. Halting.\n");
                    while (true) { tight_loop_contents(); }
                }
            }
            else { mag_recovery_attempts = 0; }
            mag_comm_errors = 0;
        }
    }

    // ── Fill + transmit record ────────────────────────────────
    log_format::Record rec;
    fill_record(rec);
    rockblock_manager::task(rec);

    loop_count++;
}

/* ============================================================
   MAIN ENTRY
   ============================================================ */

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
        sleep_ms(100);

    sleep_ms(500);
    printf("BOOTING OBC_FLIGHT...\n");
    fflush(stdout);

    bool force_recalib = wait_for_recalib_request();

    if (!system_init(force_recalib))
    {
        printf("FATAL: System init failed\n");
        fflush(stdout);
        while (true) { tight_loop_contents(); }
    }

    while (true)
    {
        if (system_initialized)
            main_loop();

        sleep_ms(1);
    }

    return 0;
}