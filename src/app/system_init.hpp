#pragma once

namespace system_init
{

bool init();
void task();
/*
system_init::init()
    ├── stdio_init_all()              ← UART/USB console
    ├── poll_sensor_recalib_requests() ← optional calibration window
    ├── sensor_manager::init()        ← IMU, mag, temp sensors
    ├── rockblock_manager::init()     ← Iridium satellite modem
    └── mission_manager::init()       ← mission state machine

*/
/* Call once after mission_manager::mission_complete() is true — safe GPIO, final log. */
void on_mission_finished();

} // namespace system_init
