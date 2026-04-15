#pragma once
#include <stdint.h>

namespace mission
{

/*
 * Mission elapsed seconds (t_sec for state machine / arbiter).
 *
 * On boot: reads last saved seconds from flash, then adds real time from
 * get_absolute_time() / to_ms_since_boot() so the count keeps moving without a 1 Hz tick.
 *
 * Call init() after persistent_store::init(). Call maybe_save() from Core 0 occasionally
 * (~main loop); it writes to flash about every 10 s of mission time.
 */
namespace mission_clock
{

bool init();

/* Mission seconds — use for ctx.t_sec and phase thresholds (450, 495, …). */
uint32_t now_seconds();

/* Same timeline, millisecond resolution. */
uint32_t now_ms();

/* Raw ms since power-on only (delays / debug). */
uint32_t millis_since_boot();

void maybe_save();

} 
} 
#pragma once
#include <stdint.h>

namespace mission
{

/*
 * Mission elapsed seconds (t_sec for state machine / arbiter).
 *
 * On boot: reads last saved seconds from flash, then adds real time from
 * get_absolute_time() / to_ms_since_boot() so the count keeps moving without a 1 Hz tick.
 *
 * Call init() after persistent_store::init(). Call maybe_save() from Core 0 occasionally
 * (~main loop); it writes to flash about every 10 s of mission time.
 */
namespace mission_clock
{

bool init();

/* Mission seconds — use for ctx.t_sec and phase thresholds (450, 495, …). */
uint32_t now_seconds();

/* Same timeline, millisecond resolution. */
uint32_t now_ms();

/* Raw ms since power-on only (delays / debug). */
uint32_t millis_since_boot();

void maybe_save();

} 
} 
