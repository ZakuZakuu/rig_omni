#include "creature_stream_protocol.h"

#include <assert.h>
#include <stdint.h>

int main() {
    uint32_t sequence = 0;
    int32_t target[CREATURE_STREAM_JOINTS] = {};
    assert(creature_stream_parse_target("creature target 7 0 -280 500 0 280\r", &sequence, target));
    assert(sequence == 7 && target[1] == -280 && target[2] == 500);
    assert(creature_stream_parse_target("creature target 8 +1 +2 +3 +4 +5", &sequence, target));
    assert(sequence == 8 && target[4] == 5);
    assert(!creature_stream_parse_target("creature target 9 0 0 0 0", &sequence, target));
    assert(!creature_stream_parse_target("creature target -1 0 0 0 0 0", &sequence, target));
    assert(!creature_stream_parse_target("creature target 10 0 0 0 0 0 trailing", &sequence, target));
    assert(!creature_stream_parse_target("mlab run 0 0 0 0 0 0 0 0 0", &sequence, target));
    return 0;
}
