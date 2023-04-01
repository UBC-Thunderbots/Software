#pragma once
#include <g3log/logmessage.hpp>
#include <chrono>
#include <list>
#include <string>
#include <unordered_map>


/**
 * Handles merging repeated log messages into a single message
 */
class LogMerger
{
   public:
    /**
     * Creates a merger to hold repeated log messages
     */
    LogMerger();

    /**
     * Returns a list of all logs that should be logged at the current time, starting
     * with the given log (if it isn't a repeat)
     * @param msg The LogMessageMover to be logged
     */
    std::list<g3::LogMessageMover> log(g3::LogMessageMover &log);

    /**
     * Adds LOG_MERGE_DURATION amount of time to the merger's currently tracked time. Used
     * for testing
     */
    void pastime();

    /**
     * Add number of repeats to a log
     */
    g3::LogMessageMover _addRepeats(g3::LogMessageMover &log, int repeats);

    /**
     * Looks through the message list for expired messages, removes them from the list and
     * map, and returns them as strings
     */
    std::list<g3::LogMessageMover> _getOldMessages(
        std::chrono::_V2::system_clock::time_point current_time);

    const std::chrono::_V2::system_clock::duration LOG_MERGE_DURATION =
        std::chrono::seconds(2);

   private:
    /**
     * Stores messages and the time they were first seen within the last few seconds
     */
    struct Message
    {
        g3::LogMessageMover log;
        std::string msg;
        std::chrono::_V2::system_clock::time_point timestamp;

        Message(g3::LogMessageMover &log, std::string msg, std::chrono::_V2::system_clock::time_point timestamp)
            : log(log), msg(msg), timestamp(timestamp)
        {
        }
    };

    std::unordered_map<std::string, int>
        repeat_map;                   // maps string messages to their number of repeats for fast access
    std::list<Message> message_list;  // used to keep track of time order for messages

    std::chrono::_V2::system_clock::duration
        passed_time;  // for testing, time passed manually
};
