#include "system_init.hpp"
#include "mission_manager.hpp"
#include "config.hpp"
#include "pico/stdlib.h"
// error with the code 1 if the init fails
int main()
{
    if (!system_init::init())
        return 1;

    while (!mission_manager::mission_complete())
    {
        system_init::task();
        sleep_ms(1000 / SAMPLE_RATE_HZ);
    }

    system_init::on_mission_finished();

    while (true)
        tight_loop_contents(); // tight_loop_contents()  =  do nothing  +  don't optimize  +  memory fence -> if want to do on both cores -> need to call explicitly for core1 as well
}
