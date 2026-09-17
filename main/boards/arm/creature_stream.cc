#include "creature_stream.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "idle_motion.h"
#include "motion_lab.h"
#include "xgo.h"
#include "xgo_action.h"

namespace {

constexpr int16_t kStreamMinCount = 100;
constexpr int16_t kStreamMaxCount = 923;
constexpr uint32_t kStreamCommandPeriodUs = 25000;
constexpr uint32_t kFeedbackFreshnessMs = 250;
constexpr uint16_t kStreamServoSpeed = 350;

struct CreatureStreamState {
    bool owned;
    bool holding;
    bool timed_out;
    bool sequence_valid;
    uint32_t last_sequence;
    uint32_t last_target_ms;
    uint32_t last_send_us;
    bool force_send;
    bool previous_idle_enabled;
    uint16_t previous_motor_speed;
    int32_t target_mdeg[MOTOR_NUM];
    int16_t target_pos[MOTOR_NUM];
};

CreatureStreamState state = {};
portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

void clear_target(CreatureStreamState* current) {
    memset(current->target_mdeg, 0, sizeof(current->target_mdeg));
    memset(current->target_pos, 0, sizeof(current->target_pos));
}

bool feedback_is_fresh(int index, uint32_t now_ms) {
    if (index < 0 || index >= MOTOR_NUM || motor[index].FbStale || motor[index].FbTimestampMs == 0) {
        return false;
    }
    return now_ms >= motor[index].FbTimestampMs &&
           now_ms - motor[index].FbTimestampMs <= kFeedbackFreshnessMs;
}

bool all_feedback_fresh(uint32_t now_ms) {
    for (int i = 0; i < MOTOR_NUM; ++i) {
        if (!feedback_is_fresh(i, now_ms)) return false;
    }
    return true;
}

int32_t position_to_mdeg(int16_t position, int index) {
    const float degrees = (static_cast<float>(position) - motor[index].ZeroPos) * M_A / M_N;
    return static_cast<int32_t>(lroundf(degrees * 1000.0f));
}

bool mdeg_to_position(int32_t mdeg, int index, int16_t* out_position) {
    if (out_position == nullptr || index < 0 || index >= MOTOR_NUM) return false;
    const float position = static_cast<float>(motor[index].ZeroPos) +
        (static_cast<float>(mdeg) / 1000.0f) * M_N / M_A;
    if (!isfinite(position)) return false;
    const int rounded = static_cast<int>(lroundf(position));
    if (rounded < kStreamMinCount || rounded > kStreamMaxCount) return false;
    *out_position = static_cast<int16_t>(rounded);
    return true;
}

void set_target_from_feedback(CreatureStreamState* current, uint32_t now_ms,
                              bool refresh_timestamp) {
    for (int i = 0; i < MOTOR_NUM; ++i) {
        if (feedback_is_fresh(i, now_ms)) {
            current->target_pos[i] = motor[i].FbPos;
            current->target_mdeg[i] = position_to_mdeg(motor[i].FbPos, i);
        }
    }
    if (refresh_timestamp) current->last_target_ms = now_ms;
    current->force_send = true;
}

}  // namespace

void creature_stream_init() {
    portENTER_CRITICAL(&state_mux);
    memset(&state, 0, sizeof(state));
    state.previous_motor_speed = kStreamServoSpeed;
    portEXIT_CRITICAL(&state_mux);
}

CreatureStreamResult creature_stream_take(uint32_t now_ms) {
    if (calibrate_mode == 1 || teach_state != TEACH_IDLE || motion_lab_is_active()) {
        return kCreatureStreamConflict;
    }
    if (!all_feedback_fresh(now_ms)) return kCreatureStreamNoFeedback;

    portENTER_CRITICAL(&state_mux);
    const bool was_owned = state.owned;
    const bool was_timed_out = state.timed_out;
    if (!was_owned) {
        state.previous_idle_enabled = idle_motion_is_enabled();
        state.previous_motor_speed = motor_speed;
        state.owned = true;
        state.holding = false;
        state.timed_out = false;
        state.sequence_valid = false;
        state.last_sequence = 0;
        state.last_send_us = 0;
        state.force_send = true;
        clear_target(&state);
    } else if (!was_timed_out && !state.holding) {
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamAlreadyOwned;
    } else {
        state.holding = false;
        state.timed_out = false;
        state.sequence_valid = false;
        state.last_sequence = 0;
        state.last_send_us = 0;
        state.force_send = true;
    }
    set_target_from_feedback(&state, now_ms, true);
    portEXIT_CRITICAL(&state_mux);

    // Ownership takes precedence over stock behavior. The first stream target
    // is exactly the current feedback posture, so taking ownership is inert.
    idle_motion_set_enable(false);
    Action_ID = 0;
    actionLoop_FLAG = 0;
    motor_speed = kStreamServoSpeed;
    return (was_owned && was_timed_out) ? kCreatureStreamAccepted : kCreatureStreamAccepted;
}

CreatureStreamResult creature_stream_accept_target(uint32_t sequence,
                                                    const int32_t target_mdeg[MOTOR_NUM],
                                                    uint32_t now_ms) {
    if (target_mdeg == nullptr) return kCreatureStreamMalformed;
    portENTER_CRITICAL(&state_mux);
    if (!state.owned) {
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamNotOwned;
    }
    if (state.timed_out) {
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamTimedOut;
    }
    if (state.sequence_valid && sequence <= state.last_sequence) {
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamStaleSequence;
    }
    int16_t positions[MOTOR_NUM] = {};
    for (int i = 0; i < MOTOR_NUM; ++i) {
        if (!mdeg_to_position(target_mdeg[i], i, &positions[i])) {
            portEXIT_CRITICAL(&state_mux);
            return kCreatureStreamInvalidTarget;
        }
    }
    for (int i = 0; i < MOTOR_NUM; ++i) {
        state.target_mdeg[i] = target_mdeg[i];
        state.target_pos[i] = positions[i];
    }
    state.sequence_valid = true;
    state.last_sequence = sequence;
    state.last_target_ms = now_ms;
    state.holding = false;
    state.force_send = true;
    portEXIT_CRITICAL(&state_mux);
    return kCreatureStreamAccepted;
}

CreatureStreamResult creature_stream_stop(uint32_t now_ms) {
    portENTER_CRITICAL(&state_mux);
    if (!state.owned) {
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamNotOwned;
    }
    if (state.timed_out) {
        state.holding = true;
        set_target_from_feedback(&state, now_ms, false);
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamTimedOut;
    }
    state.holding = true;
    set_target_from_feedback(&state, now_ms, true);
    portEXIT_CRITICAL(&state_mux);
    return kCreatureStreamAccepted;
}

CreatureStreamResult creature_stream_release() {
    portENTER_CRITICAL(&state_mux);
    if (!state.owned) {
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamNotOwned;
    }
    const bool restore_idle = state.previous_idle_enabled;
    const uint16_t restore_speed = state.previous_motor_speed;
    memset(&state, 0, sizeof(state));
    state.previous_motor_speed = restore_speed;
    portEXIT_CRITICAL(&state_mux);
    motor_speed = restore_speed;
    Action_ID = 0;
    actionLoop_FLAG = 0;
    idle_motion_set_enable(restore_idle);
    return kCreatureStreamAccepted;
}

bool creature_stream_is_owned() {
    portENTER_CRITICAL(&state_mux);
    const bool owned = state.owned;
    portEXIT_CRITICAL(&state_mux);
    return owned;
}

bool creature_stream_is_timed_out() {
    portENTER_CRITICAL(&state_mux);
    const bool timed_out = state.timed_out;
    portEXIT_CRITICAL(&state_mux);
    return timed_out;
}

void creature_stream_update(uint32_t now_us) {
    const uint32_t now_ms = now_us / 1000;
    portENTER_CRITICAL(&state_mux);
    if (state.owned && !state.holding && !state.timed_out &&
        now_ms >= state.last_target_ms &&
        now_ms - state.last_target_ms > CREATURE_STREAM_WATCHDOG_MS) {
        state.timed_out = true;
        // Prefer a fresh measured posture; otherwise retain the last valid
        // target. In either case no stock controller is allowed to take over.
        set_target_from_feedback(&state, now_ms, false);
    }
    portEXIT_CRITICAL(&state_mux);
}

bool creature_stream_should_send(uint32_t now_us) {
    portENTER_CRITICAL(&state_mux);
    if (!state.owned) {
        portEXIT_CRITICAL(&state_mux);
        return false;
    }
    const bool due = state.force_send || state.last_send_us == 0 ||
        now_us - state.last_send_us >= kStreamCommandPeriodUs;
    if (due) {
        state.force_send = false;
        state.last_send_us = now_us;
    }
    portEXIT_CRITICAL(&state_mux);
    return due;
}

void creature_stream_get_target_pos(int16_t out_pos[MOTOR_NUM]) {
    if (out_pos == nullptr) return;
    portENTER_CRITICAL(&state_mux);
    memcpy(out_pos, state.target_pos, sizeof(state.target_pos));
    portEXIT_CRITICAL(&state_mux);
}

void creature_stream_get_snapshot(CreatureStreamSnapshot* out, uint32_t now_ms) {
    if (out == nullptr) return;
    portENTER_CRITICAL(&state_mux);
    out->owned = state.owned;
    out->holding = state.holding;
    out->timed_out = state.timed_out;
    out->last_sequence = state.last_sequence;
    out->sequence_valid = state.sequence_valid;
    out->last_target_ms = state.last_target_ms;
    out->target_age_ms = (state.last_target_ms == 0 || now_ms < state.last_target_ms)
        ? UINT32_MAX : now_ms - state.last_target_ms;
    memcpy(out->target_mdeg, state.target_mdeg, sizeof(out->target_mdeg));
    memcpy(out->target_pos, state.target_pos, sizeof(out->target_pos));
    portEXIT_CRITICAL(&state_mux);
}

const char* creature_stream_result_string(CreatureStreamResult result) {
    switch (result) {
        case kCreatureStreamAccepted: return "accepted";
        case kCreatureStreamAlreadyOwned: return "already_owned";
        case kCreatureStreamConflict: return "conflict";
        case kCreatureStreamNoFeedback: return "no_feedback";
        case kCreatureStreamNotOwned: return "not_owned";
        case kCreatureStreamTimedOut: return "timed_out";
        case kCreatureStreamMalformed: return "malformed";
        case kCreatureStreamStaleSequence: return "stale_sequence";
        case kCreatureStreamInvalidTarget: return "invalid_target";
        default: return "unknown";
    }
}
