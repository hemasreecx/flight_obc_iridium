#include "mission_clock.hpp"
#include "pico/time.h"

namespace mission
{
namespace mission_clock
{

static uint32_t s0        = 0; /* mission seconds loaded from flash at init */
static uint32_t ms0       = 0; /* to_ms_since_boot snapshot at init */
static uint32_t last_save = 0; /* last mission second written to flash */

static uint32_t boot_ms()
{
    return to_ms_since_boot(get_absolute_time());
}

bool init()
{
    s0        = 0;
    ms0       = boot_ms();
    last_save = now_seconds();
    return true;
}

uint32_t now_seconds()          
{
    uint32_t d = boot_ms() - ms0;
    return s0 + (d / 1000u);
}

uint32_t now_ms()      
{
    uint32_t d = boot_ms() - ms0;
    return s0 * 1000u + d;
}

uint32_t millis_since_boot()
{
    return boot_ms();
}

void maybe_save()
{
    last_save = now_seconds();
}

} 
} 
