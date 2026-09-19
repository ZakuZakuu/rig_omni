#ifndef RIG_ARM_FEEDBACK_POLL_SCHEDULER_H
#define RIG_ARM_FEEDBACK_POLL_SCHEDULER_H

#include <stdint.h>

namespace rig_arm_feedback {

// Keep one temporarily slow servo from monopolizing the five-ID round robin.
// An attempt includes the initial request, so a value of two means one retry.
constexpr uint32_t kRequestTimeoutMs = 60;
constexpr uint8_t kMaxAttempts = 2;
constexpr uint32_t kTaskIntervalMs = 20;
constexpr uint32_t kFreshnessWindowMs = 250;

inline uint8_t next_id(uint8_t current) {
    return current >= 5 ? 1 : static_cast<uint8_t>(current + 1);
}

inline bool retry_allowed(uint8_t attempts) {
    return attempts < kMaxAttempts;
}

inline bool should_mark_stale(uint8_t attempts) {
    return attempts >= kMaxAttempts;
}

// A timeout waits at most kMaxAttempts times, with one scheduler wake-up
// between attempts. This is the worst-case time before the next ID is tried.
constexpr uint32_t worst_case_block_ms() {
    return kRequestTimeoutMs * kMaxAttempts + kTaskIntervalMs;
}

}  // namespace rig_arm_feedback

#endif  // RIG_ARM_FEEDBACK_POLL_SCHEDULER_H
