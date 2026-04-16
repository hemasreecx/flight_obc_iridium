#include "mission_manager.hpp"
#include "mission_clock.hpp"
#include "sensor_manager.hpp"
#include "config.hpp"
#include "log_format.hpp"

#include "pico/stdlib.h"

#include <stdio.h>
#include <stdint.h>

namespace mission_manager
{

static constexpr uint8_t PHASE_RING_SIZE =
    MAX_PACKETS_PER_PHASE * rockblock_manager::MAX_RECORDS_PER_PACKET;
static_assert(MAX_PACKETS_PER_PHASE * rockblock_manager::MAX_RECORDS_PER_PACKET <= 255,
              "PHASE_RING_SIZE must fit in uint8_t");

struct PhaseRing
{
    log_format::Record data[PHASE_RING_SIZE];
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t count = 0;
};

static Phase _current_phase = Phase::NONE;
static uint32_t _counter = 0;
static log_format::Record _latest_record = {};

static PhaseRing _ring_pre;
static PhaseRing _ring_blackout;
static PhaseRing _ring_post;

static bool _mission_time_done = false;
static bool _project_complete = false;
static uint32_t _tx_records_sent = 0;
static uint32_t _tx_packets_sent = 0;
static uint32_t _tx_records_dropped = 0;
static uint32_t _tx_packets_dropped = 0;

static bool ring_push(PhaseRing& ring, const log_format::Record& rec)
{
    if (ring.count >= PHASE_RING_SIZE)
        return false;

    ring.data[ring.tail] = rec;
    ring.tail = static_cast<uint8_t>((ring.tail + 1) % PHASE_RING_SIZE);
    ring.count++;
    return true;
}

static bool ring_pop_batch(PhaseRing& ring,
                           log_format::Record* out,
                           uint8_t n)
{
    if (ring.count < n)
        return false;

    for (uint8_t i = 0; i < n; i++)
    {
        out[i] = ring.data[ring.head];
        ring.head = static_cast<uint8_t>((ring.head + 1) % PHASE_RING_SIZE);
    }
    ring.count = static_cast<uint8_t>(ring.count - n);
    return true;
}

static uint8_t ring_pop_up_to(PhaseRing& ring,
                              log_format::Record* out,
                              uint8_t max_n)
{
    if (ring.count == 0)
        return 0;

    uint8_t n = (ring.count < max_n) ? ring.count : max_n;

    for (uint8_t i = 0; i < n; i++)
    {
        out[i] = ring.data[ring.head];
        ring.head = static_cast<uint8_t>((ring.head + 1) % PHASE_RING_SIZE);
    }
    ring.count = static_cast<uint8_t>(ring.count - n);
    return n;
}

static bool all_rings_empty()
{
    return (_ring_pre.count == 0) && (_ring_blackout.count == 0) && (_ring_post.count == 0);
}

static uint32_t elapsed_s()
{
    return mission::mission_clock::now_seconds();
}

static Phase compute_phase(uint32_t secs)
{
    if (secs < PHASE_PRE_DURATION_S)
        return Phase::PRE;

    if (secs < (PHASE_PRE_DURATION_S + PHASE_BLACKOUT_DURATION_S))
        return Phase::BLACKOUT;

    if (secs < PHASE_TOTAL_DURATION_S)
        return Phase::POST;

    return Phase::DONE;
}

static void account_successful_tx(uint8_t records_sent)
{
    _tx_packets_sent++;
    _tx_records_sent += records_sent;
}

static void account_dropped_tx(uint8_t records_dropped)
{
    _tx_packets_dropped++;
    _tx_records_dropped += records_dropped;
}

static void enqueue_record_for_phase(const log_format::Record& rec, Phase phase)
{
    PhaseRing* target = &_ring_pre;

    if (phase == Phase::BLACKOUT)
        target = &_ring_blackout;
    else if (phase == Phase::POST)
        target = &_ring_post;

    if (!ring_push(*target, rec))
    {
        // Ring overflow: record dropped before TX. Count it and log once per drop.
        account_dropped_tx(1);
        printf("[mission] WARN: phase ring overflow in %d, record dropped\n", (int)phase);
        fflush(stdout);
    }
}

// Normal mission-time transmit policy.
static void try_transmit_one_packet()
{
    log_format::Record batch[rockblock_manager::MAX_RECORDS_PER_PACKET];
    bool got_batch = false;

    if (_current_phase == Phase::POST)
    {
        got_batch = ring_pop_batch(_ring_blackout, batch, rockblock_manager::MAX_RECORDS_PER_PACKET) ||
                    ring_pop_batch(_ring_post,     batch, rockblock_manager::MAX_RECORDS_PER_PACKET) ||
                    ring_pop_batch(_ring_pre,      batch, rockblock_manager::MAX_RECORDS_PER_PACKET);
    }
    else if (_current_phase == Phase::BLACKOUT)
    {
        got_batch = ring_pop_batch(_ring_blackout, batch, rockblock_manager::MAX_RECORDS_PER_PACKET) ||
                    ring_pop_batch(_ring_pre,      batch, rockblock_manager::MAX_RECORDS_PER_PACKET) ||
                    ring_pop_batch(_ring_post,     batch, rockblock_manager::MAX_RECORDS_PER_PACKET);
    }
    else
    {
        got_batch = ring_pop_batch(_ring_pre, batch, rockblock_manager::MAX_RECORDS_PER_PACKET);
    }

    if (!got_batch)
        return;

    const uint8_t count = rockblock_manager::MAX_RECORDS_PER_PACKET;
    const bool ok = rockblock_manager::transmit_records(batch, count);
    if (ok)
        account_successful_tx(count);
    else
        account_dropped_tx(count);
}

// End-of-mission drain policy: BLACKOUT -> POST -> PRE.
// Uses up-to-3 records so remaining 1-2 records are not stranded.
static void drain_backlog_one_packet()
{
    log_format::Record batch[rockblock_manager::MAX_RECORDS_PER_PACKET];
    uint8_t count = 0;

    if (_ring_blackout.count > 0)
        count = ring_pop_up_to(_ring_blackout, batch, rockblock_manager::MAX_RECORDS_PER_PACKET);
    else if (_ring_post.count > 0)
        count = ring_pop_up_to(_ring_post, batch, rockblock_manager::MAX_RECORDS_PER_PACKET);
    else if (_ring_pre.count > 0)
        count = ring_pop_up_to(_ring_pre, batch, rockblock_manager::MAX_RECORDS_PER_PACKET);

    if (count == 0)
        return;

    const bool ok = rockblock_manager::transmit_records(batch, count);
    if (ok)
        account_successful_tx(count);
    else
        account_dropped_tx(count);
}

bool init()
{
    _current_phase = Phase::NONE;
    _counter = 0;
    _latest_record = {};
    _ring_pre = {};
    _ring_blackout = {};
    _ring_post = {};

    _mission_time_done = false;
    _project_complete = false;
    _tx_records_sent = 0;
    _tx_packets_sent = 0;
    _tx_records_dropped = 0;
    _tx_packets_dropped = 0;

    if (!mission::mission_clock::init())
        return false;

    _current_phase = Phase::PRE;
    return true;
}

void task()
{
    if (_project_complete)
        return;

    const Phase phase_now = compute_phase(elapsed_s());

    if (!_mission_time_done && phase_now == Phase::DONE)
    {
        _mission_time_done = true;
        _current_phase = Phase::DONE;
        printf("[mission] Mission time complete. Draining backlog BLACKOUT -> POST -> PRE...\n");
        fflush(stdout);
    }

    if (_mission_time_done)
    {
        drain_backlog_one_packet();

        if (all_rings_empty())
        {
            printf("[mission] Project done. TX sent packets=%lu, sent records=%lu, dropped packets=%lu, dropped records=%lu. Stopping.\n",
                   _tx_packets_sent,
                   _tx_records_sent,
                   _tx_packets_dropped,
                   _tx_records_dropped);
            fflush(stdout);

            _project_complete = true;
            shutdown();
        }

        mission::mission_clock::maybe_save();
        return;
    }

    if (_current_phase != phase_now)
    {
        printf("[mission] Phase change %d -> %d at %lus\n", (int)_current_phase, (int)phase_now, (unsigned long)elapsed_s());
        fflush(stdout);
        _current_phase = phase_now;
    }

    sensor_manager::task();

    const uint32_t mission_sec = mission::mission_clock::now_seconds();
    sensor_manager::fill_record(_latest_record, mission_sec);
    _counter++;

    enqueue_record_for_phase(_latest_record, phase_now);
    try_transmit_one_packet();
    mission::mission_clock::maybe_save();
}

void shutdown()
{
    rockblock_manager::shutdown();
    _current_phase = Phase::NONE;
}

Phase current_phase() { return _current_phase; }
uint32_t elapsed_seconds() { return elapsed_s(); }
uint32_t record_counter() { return _counter; }
bool mission_complete() { return _project_complete; }
uint32_t transmitted_record_count() { return _tx_records_sent; }
uint32_t transmitted_packet_count() { return _tx_packets_sent; }
uint32_t dropped_record_count() { return _tx_records_dropped; }
uint32_t dropped_packet_count() { return _tx_packets_dropped; }

} // namespace mission_manager
