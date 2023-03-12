#pragma once
#include <string>
#include <unordered_map>
#include <chrono>
#include <list>

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
         * Returns a list of all strings that should be logged at the current time, starting with the given string (if it isn't a repeat)
         * @param msg The string to be logged
        */
        std::list<std::string> log(std::string msg);

    private:
        const std::chrono::_V2::system_clock::duration LOG_MERGE_DURATION = std::chrono::seconds(2);

        /**
         * Looks through the message list for expired messages, removes them from the list and map, and returns them as strings
        */
        std::list<std::string> getOldMessages(std::chrono::_V2::system_clock::time_point current_time);

        /**
         * Add number of repeats to a message
        */
        std::string addRepeats(std::string msg, int repeats);

        /**
         * Stores messages and the time they were first seen within the last few seconds
        */
        struct Message {
            std::string message;
            std::chrono::_V2::system_clock::time_point timestamp;

            Message(std::string message, std::chrono::_V2::system_clock::time_point timestamp) : message(message), timestamp(timestamp) {}
        };

        std::unordered_map<std::string, int> repeat_map; // maps message strings to their number of repeats
        std::list<Message> message_list; // used to keep track of time order for messages
};