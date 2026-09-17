#include "creature_stream_protocol.h"

#include <ctype.h>
#include <limits.h>
#include <stdlib.h>

namespace {
constexpr uint64_t kCreatureStreamCommandPeriodUs = 25000ULL;
}

namespace {

void skip_space(const char** cursor) {
    while (**cursor != '\0' && isspace(static_cast<unsigned char>(**cursor))) ++(*cursor);
}

bool parse_unsigned(const char** cursor, uint32_t* value) {
    skip_space(cursor);
    if (**cursor == '\0' || **cursor == '-') return false;
    char* end = nullptr;
    const unsigned long parsed = strtoul(*cursor, &end, 10);
    if (end == *cursor || parsed > UINT32_MAX) return false;
    *cursor = end;
    *value = static_cast<uint32_t>(parsed);
    return true;
}

bool parse_signed(const char** cursor, int32_t* value) {
    skip_space(cursor);
    if (**cursor == '\0') return false;
    char* end = nullptr;
    const long parsed = strtol(*cursor, &end, 10);
    if (end == *cursor || parsed < INT32_MIN || parsed > INT32_MAX) return false;
    *cursor = end;
    *value = static_cast<int32_t>(parsed);
    return true;
}

}  // namespace

bool creature_stream_parse_target(const char* line, uint32_t* sequence,
                                  int32_t target_mdeg[CREATURE_STREAM_JOINTS]) {
    if (line == nullptr || sequence == nullptr || target_mdeg == nullptr) return false;
    const char* cursor = line;
    const char prefix[] = "creature target";
    const char* prefix_cursor = prefix;
    while (*prefix_cursor != '\0') {
        if (*cursor++ != *prefix_cursor++) return false;
    }
    if (*cursor != '\0' && !isspace(static_cast<unsigned char>(*cursor))) return false;
    if (!parse_unsigned(&cursor, sequence)) return false;
    for (int i = 0; i < CREATURE_STREAM_JOINTS; ++i) {
        if (!parse_signed(&cursor, &target_mdeg[i])) return false;
    }
    skip_space(&cursor);
    return *cursor == '\0';
}

bool creature_stream_command_due(uint64_t now_us, uint64_t* last_send_us,
                                 bool* force_send) {
    if (last_send_us == nullptr || force_send == nullptr) return false;
    const bool due = *force_send || *last_send_us == 0 || now_us < *last_send_us ||
        now_us - *last_send_us >= kCreatureStreamCommandPeriodUs;
    if (due) {
        *force_send = false;
        *last_send_us = now_us;
    }
    return due;
}
