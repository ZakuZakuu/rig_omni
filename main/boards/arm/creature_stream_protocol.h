#ifndef RIG_ARM_CREATURE_STREAM_PROTOCOL_H
#define RIG_ARM_CREATURE_STREAM_PROTOCOL_H

#include <stdint.h>

#define CREATURE_STREAM_JOINTS 5

// Parse exactly: creature target <uint32> <int32> x5. The function has no
// ESP-IDF or board dependencies so it can be regression-tested on the host.
bool creature_stream_parse_target(const char* line, uint32_t* sequence,
                                  int32_t target_mdeg[CREATURE_STREAM_JOINTS]);

#endif  // RIG_ARM_CREATURE_STREAM_PROTOCOL_H
