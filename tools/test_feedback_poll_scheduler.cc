#include "feedback_poll_scheduler.h"

#include <assert.h>

int main() {
    // One initial request plus one retry keeps a missing servo from consuming
    // the whole 250 ms freshness window before the next IDs are polled.
    assert(rig_arm_feedback::kMaxAttempts == 2);
    assert(rig_arm_feedback::worst_case_block_ms() < rig_arm_feedback::kFreshnessWindowMs);
    assert(rig_arm_feedback::retry_allowed(1));
    assert(!rig_arm_feedback::retry_allowed(2));
    assert(!rig_arm_feedback::retry_allowed(3));
    assert(rig_arm_feedback::should_mark_stale(2));
    assert(rig_arm_feedback::should_mark_stale(3));

    // A timeout is a skip, not a silent success: the round-robin must still
    // visit every healthy ID and wrap back to the first one.
    assert(rig_arm_feedback::next_id(1) == 2);
    assert(rig_arm_feedback::next_id(2) == 3);
    assert(rig_arm_feedback::next_id(3) == 4);
    assert(rig_arm_feedback::next_id(4) == 5);
    assert(rig_arm_feedback::next_id(5) == 1);
    return 0;
}
