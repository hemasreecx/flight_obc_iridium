#include "system_init.hpp"

#include "config.hpp"
#include "pin_config.hpp"
#include "mission_manager.hpp"
#include "sensor_manager.hpp"
#include "rockblock_manager.hpp"
#include "iridium_driver.hpp"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

namespace system_init
{

/* Per-sensor recalibration at boot (optional, within RECALIB_PROMPT_MS).
 * Multiple keys allowed — each toggles that sensor's forced recalib before init.
 *   I or 1  → KX134 accelerometer (IMU)
 *   M or 2  → QMC5883L magnetometer
 *   L or 3  → LSM6DSV80X (IMU2 / OBC temp source)
 * Legacy:   C       → all three (same as pressing I, M, L) */
static void poll_sensor_recalib_requests()
{
    printf(
        "Recalibration (optional, %ds window). Keys: I/1=KX134  M/2=QMC  L/3=LSM  C=all — then Enter optional\n",
        RECALIB_PROMPT_MS / 1000);
    fflush(stdout);

    uint32_t deadline = to_ms_since_boot(get_absolute_time()) + RECALIB_PROMPT_MS;
    while (to_ms_since_boot(get_absolute_time()) < deadline)
    {
        int ch = getchar_timeout_us(0); // this is non  - blocking , checks every 10ms if the char is given or not
        if (ch == PICO_ERROR_TIMEOUT)
        {
            sleep_ms(10);
            continue;
        }

        switch (ch)
        {
        case 'i':
        case 'I':
        case '1':
            sensor_manager::request_imu_recalib();
            printf("[system] Recalib requested: KX134 IMU\n");
            fflush(stdout);
            break;
        case 'm':
        case 'M':
        case '2':
            sensor_manager::request_mag_recalib();
            printf("[system] Recalib requested: QMC5883L magnetometer\n");
            fflush(stdout);
            break;
        case 'l':
        case 'L':
        case '3':
            sensor_manager::request_lsm_recalib();
            printf("[system] Recalib requested: LSM6DSV80X\n");
            fflush(stdout);
            break;
        case 'c':
        case 'C':
            sensor_manager::request_imu_recalib();
            sensor_manager::request_mag_recalib();
            sensor_manager::request_lsm_recalib();
            printf("[system] Recalib requested: ALL sensors (KX + QMC + LSM)\n");
            fflush(stdout);
            break;
        default:
            break;
        }
    }
}

bool init()
{
    stdio_init_all();

    poll_sensor_recalib_requests();
    // checks if sensors ok, iridium ok , mission ok

    const bool sensors_ok = sensor_manager::init();
    if (!sensors_ok)
    {
        printf("[system] ERROR: sensor_manager::init() failed (one or more subsystems not healthy)\n");
        fflush(stdout);
    }

    iridium_driver::Config iridium_cfg;
    iridium_cfg.uart_inst = IRIDIUM_UART;
    iridium_cfg.tx_pin = IRIDIUM_TX_PIN;
    iridium_cfg.rx_pin = IRIDIUM_RX_PIN;
    iridium_cfg.baud_rate = 19200;
    iridium_cfg.sleep_pin = IRIDIUM_ONOFF_PIN;
    iridium_cfg.netavb_pin = IRIDIUM_NETAVB_PIN;
    iridium_cfg.ri_pin = IRIDIUM_RI_PIN;

    const bool iridium_ok = rockblock_manager::init(iridium_cfg);
    if (!iridium_ok)
    {
        printf("[system] ERROR: rockblock_manager::init() failed (Iridium/modem not ready)\n");
        fflush(stdout);
    }

    const bool mission_ok = mission_manager::init();
    if (!mission_ok)
    {
        printf("[system] ERROR: mission_manager::init() failed\n");
        fflush(stdout);
    }

    const bool boot_ok = sensors_ok && iridium_ok && mission_ok;
    if (!boot_ok)
    {
        printf("[system] Boot ABORT: init failed (sensors=%s iridium=%s mission=%s)\n",
               sensors_ok ? "OK" : "FAIL",
               iridium_ok ? "OK" : "FAIL",
               mission_ok ? "OK" : "FAIL");
        fflush(stdout);
    }

    return boot_ok;
}

void task()
{
    mission_manager::task();
}

void on_mission_finished()
{
    printf("\n[system] === MISSION COMPLETE — entering shutdown halt ===\n");
    printf("[system] Final TX: packets sent=%lu records=%lu | dropped packets=%lu records=%lu\n",
           (unsigned long)mission_manager::transmitted_packet_count(),
           (unsigned long)mission_manager::transmitted_record_count(),
           (unsigned long)mission_manager::dropped_packet_count(),
           (unsigned long)mission_manager::dropped_record_count());
    fflush(stdout);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
}

} // namespace system_init
