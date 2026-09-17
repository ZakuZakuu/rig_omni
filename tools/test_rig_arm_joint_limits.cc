#include "ik.h"

#include <assert.h>

int main() {
    const float lower[RIG_ARM_IK_N] = {-2.62f, -1.57f, -0.50f, -1.50f, -1.20f};
    const float upper[RIG_ARM_IK_N] = {2.62f, 1.57f, 2.50f, 1.50f, 1.30f};
    for (int joint = 0; joint < RIG_ARM_IK_N; ++joint) {
        assert(rig_arm_joint_within_limits(joint, lower[joint]));
        assert(rig_arm_joint_within_limits(joint, upper[joint]));
        assert(!rig_arm_joint_within_limits(joint, lower[joint] - 1e-4f));
        assert(!rig_arm_joint_within_limits(joint, upper[joint] + 1e-4f));
    }
    assert(!rig_arm_joint_within_limits(-1, 0.0f));
    assert(!rig_arm_joint_within_limits(RIG_ARM_IK_N, 0.0f));
    return 0;
}
