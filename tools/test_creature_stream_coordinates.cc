#include "ik.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>

int main() {
    const int8_t expected_direction[RIG_ARM_IK_N] = {1, 1, -1, 1, -1};
    for (int joint = 0; joint < RIG_ARM_IK_N; ++joint) {
        assert(rig_arm_installation_direction(joint) == expected_direction[joint]);
    }
    assert(rig_arm_installation_direction(-1) == 0);
    assert(rig_arm_installation_direction(RIG_ARM_IK_N) == 0);

    // Physical feedback observed before this fix, converted from installed
    // servo space to the canonical model-space convention.
    const int32_t servo_mdeg[RIG_ARM_IK_N] = {586, -47168, -103711, 0, -72363};
    const int32_t expected_model_mdeg[RIG_ARM_IK_N] = {586, -47168, 103711, 0, 72363};
    for (int joint = 0; joint < RIG_ARM_IK_N; ++joint) {
        assert(rig_arm_servo_to_model_mdeg(servo_mdeg[joint], joint) == expected_model_mdeg[joint]);
        assert(rig_arm_model_to_servo_mdeg(expected_model_mdeg[joint], joint) == servo_mdeg[joint]);
    }

    const float q_rest_rad[RIG_ARM_IK_N] = {0.0f, -0.28f, 0.50f, 0.0f, 0.28f};
    const float expected_servo_deg[RIG_ARM_IK_N] = {0.0f, -16.042f, -28.648f, 0.0f, -16.042f};
    for (int joint = 0; joint < RIG_ARM_IK_N; ++joint) {
        const float model_deg = q_rest_rad[joint] * 180.0f / 3.14159265358979323846f;
        assert(fabsf(rig_arm_model_to_servo_deg(model_deg, joint) - expected_servo_deg[joint]) < 0.01f);
        assert(rig_arm_joint_within_limits(joint, q_rest_rad[joint]));
    }

    // ZeroPos-relative count conversion has one-count quantization at most.
    const int16_t zero_pos[RIG_ARM_IK_N] = {476, 309, 153, 527, 272};
    for (int joint = 0; joint < RIG_ARM_IK_N; ++joint) {
        const int32_t model_mdeg = static_cast<int32_t>(lroundf(q_rest_rad[joint] * 180000.0f / 3.14159265358979323846f));
        const int32_t servo_mdeg = rig_arm_model_to_servo_mdeg(model_mdeg, joint);
        const int16_t count = static_cast<int16_t>(lroundf(static_cast<float>(zero_pos[joint]) +
            (static_cast<float>(servo_mdeg) / 1000.0f) * 1024.0f / 300.0f));
        const int32_t feedback_servo_mdeg = static_cast<int32_t>(lroundf(
            (static_cast<float>(count) - zero_pos[joint]) * 300000.0f / 1024.0f));
        const int32_t feedback_model_mdeg = rig_arm_servo_to_model_mdeg(feedback_servo_mdeg, joint);
        assert(abs(feedback_model_mdeg - model_mdeg) <= 147);
    }
    return 0;
}
