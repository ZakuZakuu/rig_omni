#ifndef RIG_ARM_MOTION_LAB_H
#define RIG_ARM_MOTION_LAB_H

#include <stdbool.h>
#include <stdint.h>

#define MOTION_LAB_JOINTS 5

// Motion Lab is an explicit diagnostic path. It commands joint angles directly
// and is never used by the normal IK/action path.
enum MotionLabExperiment {
    kMotionLabSingleJointSweep = 0,
    kMotionLabSynchronizedSweep = 1,
    kMotionLabStaggeredSweep = 2,
    kMotionLabHold = 3,
    kMotionLabStepHoldReturn = 4,
};

enum MotionLabTrajectory {
    kMotionLabLinear = 0,
    kMotionLabCubicEase = 1,
    kMotionLabMinimumJerk = 2,
};

struct MotionLabConfig {
    MotionLabExperiment experiment;
    MotionLabTrajectory trajectory;
    uint8_t joint_index;  // Used by kMotionLabSingleJointSweep.
    float amplitude_deg;
    // Complete out-and-back duration for sweeps, hold duration for kMotionLabHold,
    // or one-way transition duration for kMotionLabStepHoldReturn.
    uint32_t duration_ms;
    uint32_t hold_ms;     // Peak hold time for kMotionLabStepHoldReturn.
    uint32_t stagger_ms;
    float max_velocity_deg_s;
    float max_acceleration_deg_s2;
    float deadband_deg;
};

enum MotionLabStartResult {
    kMotionLabStarted = 0,
    kMotionLabBusy,
    kMotionLabInvalidConfig,
    kMotionLabNoFeedback,
    kMotionLabUnsafeTarget,
};

struct MotionLabStatus {
    bool active;
    MotionLabExperiment experiment;
    MotionLabTrajectory trajectory;
    uint32_t elapsed_ms;
    uint32_t total_duration_ms;
    float command_deg[MOTION_LAB_JOINTS];
};

// Start from recently observed servo positions. The caller supplies raw counts
// so this module stays independent from UART/servo implementation details.
MotionLabStartResult motion_lab_start(const MotionLabConfig& config,
                                      const int16_t feedback_pos[MOTION_LAB_JOINTS],
                                      const int16_t zero_pos[MOTION_LAB_JOINTS]);

void motion_lab_stop();
bool motion_lab_is_active();

// Called from xgo_control's real-time loop. This routine allocates nothing and
// does not log or touch UART.
void motion_lab_update(uint32_t now_us);
bool motion_lab_should_send_command();
void motion_lab_get_command_deg(float out_deg[MOTION_LAB_JOINTS]);
bool motion_lab_take_finished();
void motion_lab_get_status(MotionLabStatus* out_status, uint32_t now_us);

const char* motion_lab_start_result_string(MotionLabStartResult result);

#endif  // RIG_ARM_MOTION_LAB_H
