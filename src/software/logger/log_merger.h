#pragma once
#include <string>
#include <unordered_map>
#include <chrono>

/**
 * Handles merging repeated log messages into a single message
*/
class LogMerger {
    public:
        /**
         * Creates a merger to hold repeated log messages
        */
        LogMerger();

        /**
         * If the message has been seen in the last few seconds, increase its repeats and return true
         * Otherwise, add it to the tracked messages and return false
         * @param msg The message to be logged
        */
        bool isMessageRepeated(std::string msg);

        /**
         * Returns the message with its number of repeats appended
         * If not in the tracked messages, return the message as is
         * @param msg The message to be logged
        */
        std::string addRepeats(std::string msg);

    private:
        const std::chrono::_V2::system_clock::duration LOG_MERGE_DURATION = std::chrono::seconds(2);

        struct Message {
            int repeats;
            std::chrono::_V2::system_clock::time_point timestamp;

            Message(std::chrono::_V2::system_clock::time_point time) : repeats(0), timestamp(time) {}
        };

        std::unordered_map<std::string, int> message_map;
};