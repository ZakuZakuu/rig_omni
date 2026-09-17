#ifndef RIG_ARM_CREATURE_STREAM_H
#define RIG_ARM_CREATURE_STREAM_H

#include <stdbool.h>
#include <stdint.h>

#include "creature_stream_protocol.h"

#define CREATURE_STREAM_PROTOCOL 1
#define CREATURE_STREAM_MAX_HZ 40
#define CREATURE_STREAM_WATCHDOG_MS 250

enum CreatureStreamResult {
    kCreatureStreamAccepted = 0,
    kCreatureStreamAlreadyOwned,
    kCreatureStreamConflict,
    kCreatureStreamNoFeedback,
    kCreatureStreamNotOwned,
    kCreatureStreamTimedOut,
    kCreatureStreamMalformed,
    kCreatureStreamStaleSequence,
    kCreatureStreamInvalidTarget,
    kCreatureStreamFaultHold,
};

struct CreatureStreamSnapshot {
    bool owned;
    bool holding;
    bool timed_out;
    bool fault_hold;
    uint32_t last_sequence;
    bool sequence_valid;
    uint32_t target_age_ms;
    uint64_t last_target_us;
    int32_t target_mdeg[CREATURE_STREAM_JOINTS];
    int16_t target_pos[CREATURE_STREAM_JOINTS];
    int16_t feedback_pos[CREATURE_STREAM_JOINTS];
    int32_t feedback_mdeg[CREATURE_STREAM_JOINTS];
    uint32_t feedback_age_ms[CREATURE_STREAM_JOINTS];
    bool feedback_stale[CREATURE_STREAM_JOINTS];
    uint8_t servo_error[CREATURE_STREAM_JOINTS];
    float voltage_v;
};

void creature_stream_init();
CreatureStreamResult creature_stream_take(uint64_t now_us);
CreatureStreamResult creature_stream_accept_target(uint32_t sequence,
                                                    const int32_t target_mdeg[CREATURE_STREAM_JOINTS],
                                                    uint64_t now_us);
CreatureStreamResult creature_stream_stop(uint64_t now_us);
CreatureStreamResult creature_stream_release();

bool creature_stream_is_owned();
bool creature_stream_is_timed_out();
void creature_stream_update(uint64_t now_us);
bool creature_stream_should_send(uint64_t now_us);
void creature_stream_get_target_pos(int16_t out_pos[CREATURE_STREAM_JOINTS]);
void creature_stream_get_snapshot(CreatureStreamSnapshot* out, uint64_t now_us);
bool creature_stream_feedback_is_healthy(uint64_t now_us);
const char* creature_stream_result_string(CreatureStreamResult result);

#endif  // RIG_ARM_CREATURE_STREAM_H
