#include "motion_lab.h"

#include <math.h>
#include <string.h>

namespace {

constexpr float kCountsPerDeg = 1024.0f / 300.0f;
constexpr int16_t kExperimentMinCount = 100;
constexpr int16_t kExperimentMaxCount = 923;
// Keep trajectory integration at the 2 ms control cadence, but limit actual
// Keep the command cadence off the 20 ms feedback-poll period. The resulting
// 40 Hz sync writes retain bus bandwidth and avoid a persistent phase collision.
constexpr uint32_t kCommandPeriodUs = 25000;
constexpr uint32_t kKeepaliveUs = 100000;

struct MotionLabState {
    bool active;
    bool finished;
    bool command_dirty;
    MotionLabConfig config;
    uint32_t started_us;
    uint32_t last_update_us;
    uint32_t last_send_us;
    float baseline_deg[MOTION_LAB_JOINTS];
    float command_deg[MOTION_LAB_JOINTS];
    float command_velocity_deg_s[MOTION_LAB_JOINTS];
    float sent_deg[MOTION_LAB_JOINTS];
};

MotionLabState state = {};

float clampf(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

float easing(float normalized, MotionLabTrajectory trajectory) {
    const float t = clampf(normalized, 0.0f, 1.0f);
    switch (trajectory) {
        case kMotionLabLinear:
            return t;
        case kMotionLabCubicEase:
            return t * t * (3.0f - 2.0f * t);
        case kMotionLabMinimumJerk:
            return t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
        default:
            return t;
    }
}

float out_and_back(uint32_t elapsed_ms, uint32_t duration_ms, MotionLabTrajectory trajectory) {
    if (duration_ms == 0 || elapsed_ms >= duration_ms) return 0.0f;
    const float phase = static_cast<float>(elapsed_ms) / static_cast<float>(duration_ms);
    if (phase <= 0.5f) return easing(phase * 2.0f, trajectory);
    return easing((1.0f - phase) * 2.0f, trajectory);
}

float step_hold_return(uint32_t elapsed_ms, uint32_t transition_ms, uint32_t hold_ms,
                       MotionLabTrajectory trajectory) {
    if (transition_ms == 0) return 0.0f;
    if (elapsed_ms < transition_ms) {
        return easing(static_cast<float>(elapsed_ms) / transition_ms, trajectory);
    }
    if (elapsed_ms < transition_ms + hold_ms) return 1.0f;
    const uint32_t return_elapsed_ms = elapsed_ms - transition_ms - hold_ms;
    if (return_elapsed_ms >= transition_ms) return 0.0f;
    return 1.0f - easing(static_cast<float>(return_elapsed_ms) / transition_ms, trajectory);
}

bool has_safe_targets(const MotionLabConfig& config,
                      const int16_t feedback_pos[MOTION_LAB_JOINTS]) {
    for (int i = 0; i < MOTION_LAB_JOINTS; ++i) {
        if (feedback_pos[i] < 1 || feedback_pos[i] > 1023) return false;
        const bool moves = config.experiment == kMotionLabSynchronizedSweep ||
                           config.experiment == kMotionLabStaggeredSweep ||
                           ((config.experiment == kMotionLabSingleJointSweep ||
                             config.experiment == kMotionLabStepHoldReturn) &&
                            i == config.joint_index);
        if (!moves) continue;
        const int target = feedback_pos[i] + static_cast<int>(config.amplitude_deg * kCountsPerDeg);
        if (target < kExperimentMinCount || target > kExperimentMaxCount) return false;
    }
    return true;
}

bool valid_config(const MotionLabConfig& config) {
    if (config.experiment < kMotionLabSingleJointSweep || config.experiment > kMotionLabStepHoldReturn ||
        config.trajectory < kMotionLabLinear || config.trajectory > kMotionLabMinimumJerk ||
        config.joint_index >= MOTION_LAB_JOINTS || config.duration_ms < 500 || config.duration_ms > 30000 ||
        config.hold_ms > 10000 || config.stagger_ms > 2000 || config.max_velocity_deg_s <= 0.0f ||
        config.max_acceleration_deg_s2 <= 0.0f || config.deadband_deg < 0.0f ||
        config.deadband_deg > 2.0f) {
        return false;
    }
    if (config.experiment == kMotionLabHold) return config.amplitude_deg == 0.0f;
    return config.amplitude_deg >= 1.0f && config.amplitude_deg <= 10.0f;
}

uint32_t total_duration_ms() {
    if (state.config.experiment == kMotionLabStepHoldReturn) {
        return state.config.duration_ms * 2 + state.config.hold_ms;
    }
    if (state.config.experiment != kMotionLabStaggeredSweep) return state.config.duration_ms;
    return state.config.duration_ms + state.config.stagger_ms * (MOTION_LAB_JOINTS - 1);
}

}  // namespace

MotionLabStartResult motion_lab_start(const MotionLabConfig& config,
                                      const int16_t feedback_pos[MOTION_LAB_JOINTS],
                                      const int16_t zero_pos[MOTION_LAB_JOINTS]) {
    if (state.active) return kMotionLabBusy;
    if (!valid_config(config)) return kMotionLabInvalidConfig;
    if (!has_safe_targets(config, feedback_pos)) {
        for (int i = 0; i < MOTION_LAB_JOINTS; ++i) {
            if (feedback_pos[i] < 1 || feedback_pos[i] > 1023) return kMotionLabNoFeedback;
        }
        return kMotionLabUnsafeTarget;
    }

    memset(&state, 0, sizeof(state));
    state.config = config;
    for (int i = 0; i < MOTION_LAB_JOINTS; ++i) {
        state.baseline_deg[i] = (feedback_pos[i] - zero_pos[i]) / kCountsPerDeg;
        state.command_deg[i] = state.baseline_deg[i];
        state.sent_deg[i] = state.baseline_deg[i];
    }
    state.active = true;
    state.command_dirty = true;
    return kMotionLabStarted;
}

void motion_lab_stop() {
    if (!state.active) return;
    for (int i = 0; i < MOTION_LAB_JOINTS; ++i) {
        state.command_deg[i] = state.baseline_deg[i];
        state.command_velocity_deg_s[i] = 0.0f;
    }
    state.active = false;
    state.finished = true;
    state.command_dirty = true;
}

bool motion_lab_is_active() {
    return state.active;
}

void motion_lab_update(uint32_t now_us) {
    if (!state.active) return;
    if (state.started_us == 0) {
        state.started_us = now_us;
        state.last_update_us = now_us;
        return;
    }

    const uint32_t elapsed_ms = (now_us - state.started_us) / 1000;
    if (elapsed_ms >= total_duration_ms()) {
        for (int i = 0; i < MOTION_LAB_JOINTS; ++i) {
            state.command_deg[i] = state.baseline_deg[i];
        }
        state.command_dirty = true;
        state.active = false;
        state.finished = true;
        return;
    }

    float dt_s = (now_us - state.last_update_us) / 1000000.0f;
    state.last_update_us = now_us;
    if (dt_s <= 0.0f || dt_s > 0.02f) dt_s = 0.002f;

    bool changed = false;
    for (int i = 0; i < MOTION_LAB_JOINTS; ++i) {
        float desired = state.baseline_deg[i];
        if (state.config.experiment != kMotionLabHold) {
            bool moves = state.config.experiment == kMotionLabSynchronizedSweep ||
                         state.config.experiment == kMotionLabStaggeredSweep ||
                         ((state.config.experiment == kMotionLabSingleJointSweep ||
                           state.config.experiment == kMotionLabStepHoldReturn) &&
                          i == state.config.joint_index);
            if (moves) {
                uint32_t local_ms = elapsed_ms;
                if (state.config.experiment == kMotionLabStaggeredSweep) {
                    const uint32_t start_ms = state.config.stagger_ms * i;
                    local_ms = elapsed_ms > start_ms ? elapsed_ms - start_ms : 0;
                }
                if (state.config.experiment == kMotionLabStepHoldReturn) {
                    desired += state.config.amplitude_deg * step_hold_return(
                        local_ms, state.config.duration_ms, state.config.hold_ms, state.config.trajectory);
                } else {
                    desired += state.config.amplitude_deg * out_and_back(
                        local_ms, state.config.duration_ms, state.config.trajectory);
                }
            }
        }

        const float velocity_target = clampf((desired - state.command_deg[i]) / dt_s,
                                             -state.config.max_velocity_deg_s,
                                             state.config.max_velocity_deg_s);
        const float max_velocity_step = state.config.max_acceleration_deg_s2 * dt_s;
        state.command_velocity_deg_s[i] += clampf(velocity_target - state.command_velocity_deg_s[i],
                                                   -max_velocity_step, max_velocity_step);
        const float next = state.command_deg[i] + state.command_velocity_deg_s[i] * dt_s;
        // Keep integrating the internal trajectory on every control tick.
        // Deadband applies only to bus writes below; resetting this value to
        // sent_deg here would prevent sub-deadband 2 ms steps from accumulating.
        state.command_deg[i] = next;
        if (fabsf(next - state.sent_deg[i]) >= state.config.deadband_deg) {
            changed = true;
        }
    }
    state.command_dirty = state.command_dirty || changed;

    // Deadband avoids rewriting unchanged targets. A low-rate keepalive still
    // reapplies the requested position after a transient bus-side loss.
    if (state.last_send_us == 0 || now_us - state.last_send_us >= kKeepaliveUs) {
        state.command_dirty = true;
    }
}

bool motion_lab_should_send_command() {
    if (!state.command_dirty) return false;
    if (state.last_send_us != 0 &&
        state.last_update_us - state.last_send_us < kCommandPeriodUs) {
        return false;
    }
    return true;
}

void motion_lab_get_command_deg(float out_deg[MOTION_LAB_JOINTS]) {
    for (int i = 0; i < MOTION_LAB_JOINTS; ++i) {
        out_deg[i] = state.command_deg[i];
        state.sent_deg[i] = state.command_deg[i];
    }
    state.command_dirty = false;
    state.last_send_us = state.last_update_us;
}

bool motion_lab_take_finished() {
    const bool finished = state.finished;
    state.finished = false;
    return finished;
}

void motion_lab_get_status(MotionLabStatus* out_status, uint32_t now_us) {
    if (out_status == nullptr) return;
    out_status->active = state.active;
    out_status->experiment = state.config.experiment;
    out_status->trajectory = state.config.trajectory;
    out_status->elapsed_ms = state.started_us == 0 ? 0 : (now_us - state.started_us) / 1000;
    out_status->total_duration_ms = total_duration_ms();
    for (int i = 0; i < MOTION_LAB_JOINTS; ++i) {
        out_status->command_deg[i] = state.command_deg[i];
    }
}

const char* motion_lab_start_result_string(MotionLabStartResult result) {
    switch (result) {
        case kMotionLabStarted: return "started";
        case kMotionLabBusy: return "busy";
        case kMotionLabInvalidConfig: return "invalid config";
        case kMotionLabNoFeedback: return "waiting for all joint feedback";
        case kMotionLabUnsafeTarget: return "target outside conservative diagnostic range";
        default: return "unknown";
    }
}
