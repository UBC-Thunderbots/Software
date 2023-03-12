#include "software/logger/log_merger.h"

LogMerger::LogMerger() {}

bool LogMerger::isMessageRepeated(std::string msg) {
    // not in map: add to map and return false
    // in map, but outside time: replace in map and return false
    // in map, and within time: increment repeats and return true

    if (message_map.count(msg)) {
        Message& m = message_map[msg];
        std::chrono::_V2::system_clock::time_point current_time =
            std::chrono::system_clock::now();
        if (current_time - LOG_MERGE_DURATION < m.timestamp) {
            // repeated message within time
            m.repeats++;
        }
    }
    // either not in map or too old
    
}