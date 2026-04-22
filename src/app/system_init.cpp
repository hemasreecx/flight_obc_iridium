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
#include <string.h>

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
        "Recalibration (optional, %ds window). Commands: cal_kx | cal_qmc | cal_lsm | cal (then Enter)\n",
        RECALIB_PROMPT_MS / 1000);
    fflush(stdout);

    auto apply_command = [](const char* cmd)
    {
        if (strcmp(cmd, "cal_kx") == 0 || strcmp(cmd, "CAL_KX") == 0)
        {
            sensor_manager::request_imu_recalib();
            printf("[system] Recalib requested: KX134 IMU\n");
            fflush(stdout);
            return;
        }

        if (strcmp(cmd, "cal_qmc") == 0 || strcmp(cmd, "CAL_QMC") == 0)
        {
            sensor_manager::request_mag_recalib();
            printf("[system] Recalib requested: QMC5883L magnetometer\n");
            fflush(stdout);
            return;
        }

        if (strcmp(cmd, "cal_lsm") == 0 || strcmp(cmd, "CAL_LSM") == 0)
        {
            sensor_manager::request_lsm_recalib();
            printf("[system] Recalib requested: LSM6DSV80X\n");
            fflush(stdout);
            return;
        }

        if (strcmp(cmd, "cal") == 0 || strcmp(cmd, "CAL") == 0)
        {
            sensor_manager::request_imu_recalib();
            sensor_manager::request_mag_recalib();
            sensor_manager::request_lsm_recalib();
            printf("[system] Recalib requested: ALL sensors (KX + QMC + LSM)\n");
            fflush(stdout);
            return;
        }
    };

    const uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    uint32_t deadline = start_ms + RECALIB_PROMPT_MS;
    int32_t last_announced_remaining_s = -1;
    char command[24] = {0};
    uint8_t idx = 0;
    while (to_ms_since_boot(get_absolute_time()) < deadline)
    {
        const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        const uint32_t elapsed_ms = now_ms - start_ms;
        const int32_t remaining_s =
            (int32_t)((RECALIB_PROMPT_MS > elapsed_ms)
                          ? ((RECALIB_PROMPT_MS - elapsed_ms + 999) / 1000)
                          : 0);
        if (remaining_s != last_announced_remaining_s)
        {
            last_announced_remaining_s = remaining_s;
            printf("[system] recalib window: %lds left (type cal_kx | cal_qmc | cal_lsm | cal + Enter)\n",
                   (long)remaining_s);
            fflush(stdout);
        }

        int ch = getchar_timeout_us(0); // non-blocking poll
        if (ch == PICO_ERROR_TIMEOUT)
        {
            sleep_ms(10);
            continue;
        }

        if (ch == '\r' || ch == '\n')
        {
            if (idx > 0)
            {
                command[idx] = '\0';
                apply_command(command);
                idx = 0;
                memset(command, 0, sizeof(command));
            }
            continue;
        }

        if (idx < sizeof(command) - 1)
        {
            command[idx++] = (char)ch;
        }
    }

    // Process any partially typed command when window closes.
    if (idx > 0)
    {
        command[idx] = '\0';
        apply_command(command);
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
    iridium_cfg.baud_rate = IRIDIUM_BAUD_RATE;
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
