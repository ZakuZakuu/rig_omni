#include "creature_stream.h"

#include <limits.h>
#include <math.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#include "idle_motion.h"
#include "ik.h"
#include "motion_lab.h"
#include "xgo.h"
#include "xgo_action.h"

namespace {

constexpr int16_t kStreamMinCount = 100;
constexpr int16_t kStreamMaxCount = 923;
constexpr uint64_t kFeedbackFreshnessUs = 250000ULL;
constexpr uint16_t kStreamServoSpeed = 350;

struct CreatureStreamState {
    bool owned;
    bool holding;
    bool timed_out;
    bool fault_hold;
    bool sequence_valid;
    uint32_t last_sequence;
    uint64_t last_target_us;
    uint64_t last_send_us;
    bool force_send;
    bool previous_idle_enabled;
    uint16_t previous_motor_speed;
    int32_t target_mdeg[MOTOR_NUM];
    int16_t target_pos[MOTOR_NUM];
};

CreatureStreamState state = {};
CreatureStreamFaultSnapshot fault_snapshot = {};
bool fault_snapshot_pending = false;
portMUX_TYPE state_mux = portMUX_INITIALIZER_UNLOCKED;

uint64_t feedback_age_us(int index, uint64_t now_us) {
    if (index < 0 || index >= MOTOR_NUM || motor[index].FbTimestampUs == 0 ||
        now_us < motor[index].FbTimestampUs) {
        return UINT64_MAX;
    }
    return now_us - motor[index].FbTimestampUs;
}

bool feedback_is_fresh(int index, uint64_t now_us) {
    return index >= 0 && index < MOTOR_NUM && !motor[index].FbStale &&
           motor[index].FbTimestampUs != 0 && feedback_age_us(index, now_us) <= kFeedbackFreshnessUs;
}

bool all_feedback_fresh(uint64_t now_us) {
    for (int i = 0; i < MOTOR_NUM; ++i) {
        if (!feedback_is_fresh(i, now_us)) return false;
    }
    return true;
}

bool all_feedback_healthy(uint64_t now_us) {
    if (!all_feedback_fresh(now_us)) return false;
    for (int i = 0; i < MOTOR_NUM; ++i) {
        if (motor[i].FbError != 0) return false;
    }
    return true;
}

int32_t position_to_mdeg(int16_t position, int index) {
    const float servo_degrees =
        (static_cast<float>(position) - motor[index].ZeroPos) * M_A / M_N;
    const int32_t servo_mdeg = static_cast<int32_t>(lroundf(servo_degrees * 1000.0f));
    return rig_arm_servo_to_model_mdeg(servo_mdeg, index);
}

bool mdeg_to_position(int32_t mdeg, int index, int16_t* out_position) {
    if (out_position == nullptr || index < 0 || index >= MOTOR_NUM) return false;
    const float radians = static_cast<float>(mdeg) * PI / 180000.0f;
    if (!rig_arm_joint_within_limits(index, radians)) return false;
    const int32_t servo_mdeg = rig_arm_model_to_servo_mdeg(mdeg, index);
    const float position = static_cast<float>(motor[index].ZeroPos) +
        (static_cast<float>(servo_mdeg) / 1000.0f) * M_N / M_A;
    if (!isfinite(position)) return false;
    const int rounded = static_cast<int>(lroundf(position));
    if (rounded < kStreamMinCount || rounded > kStreamMaxCount) return false;
    *out_position = static_cast<int16_t>(rounded);
    return true;
}

void clear_target(CreatureStreamState* current) {
    memset(current->target_mdeg, 0, sizeof(current->target_mdeg));
    memset(current->target_pos, 0, sizeof(current->target_pos));
}

void set_target_from_feedback(CreatureStreamState* current, uint64_t now_us, bool force_send) {
    for (int i = 0; i < MOTOR_NUM; ++i) {
        if (feedback_is_fresh(i, now_us)) {
            current->target_pos[i] = motor[i].FbPos;
            current->target_mdeg[i] = position_to_mdeg(motor[i].FbPos, i);
        }
    }
    current->force_send = force_send;
}

void capture_fault_snapshot(uint64_t now_us) {
    fault_snapshot.timestamp_ms = static_cast<uint32_t>(now_us / 1000ULL);
    fault_snapshot.last_sequence = state.last_sequence;
    for (int i = 0; i < MOTOR_NUM; ++i) {
        fault_snapshot.feedback_pos[i] = motor[i].FbPos;
        const uint64_t age_us = feedback_age_us(i, now_us);
        fault_snapshot.feedback_age_ms[i] = age_us == UINT64_MAX
            ? UINT32_MAX : static_cast<uint32_t>(age_us / 1000ULL);
        fault_snapshot.feedback_stale[i] = motor[i].FbStale;
        fault_snapshot.servo_error[i] = motor[i].FbError;
    }
    XgoFeedbackPollSnapshot poll = {};
    xgo_feedback_poll_get_snapshot(&poll);
    fault_snapshot.poll_id = poll.poll_id;
    fault_snapshot.poll_pending = poll.request_pending;
    fault_snapshot.poll_attempts = poll.attempts;
    for (int i = 0; i < MOTOR_NUM; ++i) {
        fault_snapshot.poll_skip_count[i] = poll.skip_count[i];
    }
    XgoFeedbackDiagnostics diagnostics = {};
    xgo_feedback_poll_get_diagnostics(&diagnostics);
    for (int i = 0; i < MOTOR_NUM; ++i) {
        fault_snapshot.poll_request_count[i] = diagnostics.request_count[i];
        fault_snapshot.poll_valid_response_count[i] = diagnostics.valid_response_count[i];
        fault_snapshot.poll_timeout_count[i] = diagnostics.timeout_count[i];
    }
    fault_snapshot.checksum_invalid_count = diagnostics.checksum_invalid_count;
    fault_snapshot.malformed_packet_count = diagnostics.malformed_packet_count;
    fault_snapshot.unexpected_response_count = diagnostics.unexpected_response_count;
    fault_snapshot.bus_overlap_count = diagnostics.bus_overlap_count;
    fault_snapshot.deferred_command_count = diagnostics.deferred_command_count;
    fault_snapshot.bus_overlap_pending_id = diagnostics.bus_overlap_pending_id;
    fault_snapshot.bus_overlap_pending_age_ms = diagnostics.bus_overlap_pending_age_ms;
    fault_snapshot.bus_overlap_last_sequence = diagnostics.bus_overlap_last_sequence;
    fault_snapshot_pending = true;
}

}  // namespace

void creature_stream_init() {
    portENTER_CRITICAL(&state_mux);
    memset(&state, 0, sizeof(state));
    memset(&fault_snapshot, 0, sizeof(fault_snapshot));
    fault_snapshot_pending = false;
    state.previous_motor_speed = kStreamServoSpeed;
    portEXIT_CRITICAL(&state_mux);
}

CreatureStreamResult creature_stream_take(uint64_t now_us) {
    if (calibrate_mode == 1 || teach_state != TEACH_IDLE || motion_lab_is_active()) {
        return kCreatureStreamConflict;
    }
    if (!all_feedback_healthy(now_us)) return kCreatureStreamNoFeedback;

    portENTER_CRITICAL(&state_mux);
    const bool was_owned = state.owned;
    const bool was_timed_out = state.timed_out;
    const bool was_fault_hold = state.fault_hold;
    if (was_owned && !state.holding && !was_timed_out && !was_fault_hold) {
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamAlreadyOwned;
    }
    if (!was_owned) {
        state.previous_idle_enabled = idle_motion_is_enabled();
        state.previous_motor_speed = motor_speed;
    }
    state.owned = true;
    state.holding = true;
    state.timed_out = false;
    state.fault_hold = false;
    state.sequence_valid = false;
    state.last_sequence = 0;
    state.last_target_us = 0;
    state.last_send_us = 0;
    clear_target(&state);
    set_target_from_feedback(&state, now_us, true);
    portEXIT_CRITICAL(&state_mux);

    idle_motion_set_enable(false);
    Action_ID = 0;
    actionLoop_FLAG = 0;
    motor_speed = kStreamServoSpeed;
    return (was_owned && (was_timed_out || was_fault_hold))
        ? kCreatureStreamAccepted : (was_owned ? kCreatureStreamAlreadyOwned : kCreatureStreamAccepted);
}

CreatureStreamResult creature_stream_accept_target(uint32_t sequence,
                                                    const int32_t target_mdeg[MOTOR_NUM],
                                                    uint64_t now_us) {
    if (target_mdeg == nullptr) return kCreatureStreamMalformed;
    portENTER_CRITICAL(&state_mux);
    if (!state.owned) {
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamNotOwned;
    }
    if (state.timed_out || state.fault_hold) {
        portEXIT_CRITICAL(&state_mux);
        return state.fault_hold ? kCreatureStreamFaultHold : kCreatureStreamTimedOut;
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
    state.last_target_us = now_us;
    state.holding = false;
    // Normal stream targets update the latest-target buffer only.  The
    // physical command cadence remains enforced by creature_stream_should_send.
    state.force_send = false;
    portEXIT_CRITICAL(&state_mux);
    return kCreatureStreamAccepted;
}

CreatureStreamResult creature_stream_stop(uint64_t now_us) {
    portENTER_CRITICAL(&state_mux);
    if (!state.owned) {
        portEXIT_CRITICAL(&state_mux);
        return kCreatureStreamNotOwned;
    }
    state.holding = true;
    set_target_from_feedback(&state, now_us, true);
    const bool fault = state.fault_hold;
    const bool timed_out = state.timed_out;
    portEXIT_CRITICAL(&state_mux);
    if (fault) return kCreatureStreamFaultHold;
    if (timed_out) return kCreatureStreamTimedOut;
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

void creature_stream_update(uint64_t now_us) {
    portENTER_CRITICAL(&state_mux);
    if (state.owned && !state.holding && !state.timed_out && !state.fault_hold && state.sequence_valid) {
        if (!all_feedback_healthy(now_us)) {
            if (!state.fault_hold) {
                capture_fault_snapshot(now_us);
            }
            state.fault_hold = true;
            state.holding = true;
            set_target_from_feedback(&state, now_us, true);
        } else if (now_us >= state.last_target_us &&
                   now_us - state.last_target_us >
                       static_cast<uint64_t>(CREATURE_STREAM_WATCHDOG_MS) * 1000ULL) {
            state.timed_out = true;
            state.holding = true;
            set_target_from_feedback(&state, now_us, true);
        }
    }
    portEXIT_CRITICAL(&state_mux);
}

bool creature_stream_should_send(uint64_t now_us) {
    portENTER_CRITICAL(&state_mux);
    if (!state.owned) {
        portEXIT_CRITICAL(&state_mux);
        return false;
    }
    const bool due = creature_stream_command_due(now_us, &state.last_send_us, &state.force_send);
    portEXIT_CRITICAL(&state_mux);
    return due;
}

void creature_stream_get_target_pos(int16_t out_pos[MOTOR_NUM]) {
    if (out_pos == nullptr) return;
    portENTER_CRITICAL(&state_mux);
    memcpy(out_pos, state.target_pos, sizeof(state.target_pos));
    portEXIT_CRITICAL(&state_mux);
}

bool creature_stream_feedback_is_healthy(uint64_t now_us) {
    return all_feedback_healthy(now_us);
}

void creature_stream_get_snapshot(CreatureStreamSnapshot* out, uint64_t now_us) {
    if (out == nullptr) return;
    portENTER_CRITICAL(&state_mux);
    out->owned = state.owned;
    out->holding = state.holding;
    out->timed_out = state.timed_out;
    out->fault_hold = state.fault_hold;
    out->last_sequence = state.last_sequence;
    out->sequence_valid = state.sequence_valid;
    out->last_target_us = state.last_target_us;
    out->target_age_ms = (state.last_target_us == 0 || now_us < state.last_target_us)
        ? UINT32_MAX : static_cast<uint32_t>((now_us - state.last_target_us) / 1000ULL);
    memcpy(out->target_mdeg, state.target_mdeg, sizeof(out->target_mdeg));
    memcpy(out->target_pos, state.target_pos, sizeof(out->target_pos));
    for (int i = 0; i < MOTOR_NUM; ++i) {
        out->feedback_pos[i] = motor[i].FbPos;
        out->feedback_mdeg[i] = position_to_mdeg(motor[i].FbPos, i);
        const uint64_t age_us = feedback_age_us(i, now_us);
        out->feedback_age_ms[i] = age_us == UINT64_MAX
            ? UINT32_MAX : static_cast<uint32_t>(age_us / 1000ULL);
        out->feedback_stale[i] = motor[i].FbStale;
        out->servo_error[i] = motor[i].FbError;
    }
    out->voltage_v = servo_voltage;
    portEXIT_CRITICAL(&state_mux);
}

bool creature_stream_take_fault_snapshot(CreatureStreamFaultSnapshot* out) {
    if (out == nullptr) return false;
    portENTER_CRITICAL(&state_mux);
    if (!fault_snapshot_pending) {
        portEXIT_CRITICAL(&state_mux);
        return false;
    }
    memcpy(out, &fault_snapshot, sizeof(*out));
    fault_snapshot_pending = false;
    portEXIT_CRITICAL(&state_mux);
    return true;
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
        case kCreatureStreamFaultHold: return "fault_hold";
        default: return "unknown";
    }
}
