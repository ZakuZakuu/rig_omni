#ifndef RIG_ARM_CREATURE_STREAM_PROTOCOL_H
#define RIG_ARM_CREATURE_STREAM_PROTOCOL_H

#include <stdint.h>

#define CREATURE_STREAM_JOINTS 5

// Parse exactly: creature target <uint32> <int32> x5. The function has no
// ESP-IDF or board dependencies so it can be regression-tested on the host.
bool creature_stream_parse_target(const char* line, uint32_t* sequence,
                                  int32_t target_mdeg[CREATURE_STREAM_JOINTS]);

// Pure command-cadence primitive shared by the firmware state machine tests.
// ``force_send`` is reserved for acquisition/safety HOLD, not normal targets.
bool creature_stream_command_due(uint64_t now_us, uint64_t* last_send_us,
                                 bool* force_send);

#endif  // RIG_ARM_CREATURE_STREAM_PROTOCOL_H
