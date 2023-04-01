#include "software/logger/log_merger.h"

LogMerger::LogMerger() : passed_time(std::chrono::seconds(0)) {}

std::list<g3::LogMessageMover> LogMerger::log(g3::LogMessageMover &log)
{
    std::string msg = log.get().message();

    std::chrono::_V2::system_clock::time_point current_time =
        std::chrono::system_clock::now();
    // add passed time from testing
    current_time += passed_time;
    std::list<g3::LogMessageMover> messages_to_log = _getOldMessages(current_time);

    if (repeat_map.count(msg))
    {
        // msg is in the repeat map, add a repeat and return the old messages
        repeat_map[msg]++;
        return messages_to_log;
    }

    // msg is not in the repeat map, add the log to the map and list and log it
    repeat_map[msg] = 0;
    message_list.push_back(Message(log, msg, current_time));

    messages_to_log.push_front(log);
    return messages_to_log;
}

std::list<g3::LogMessageMover> LogMerger::_getOldMessages(
    std::chrono::_V2::system_clock::time_point current_time)
{
    std::list<g3::LogMessageMover> result;
    while (message_list.size() > 0)
    {
        Message current_message = message_list.front();
        if (current_time - LOG_MERGE_DURATION >= current_message.timestamp)
        {
            // old, add repeats to the message and add it to the list
            g3::LogMessageMover log = current_message.log;
            result.push_back(_addRepeats(log, repeat_map[current_message.msg]));
            repeat_map.erase(current_message.msg);
            message_list.pop_front();
        }
        else
        {
            // new message, rest of list is new
            break;
        }
    }
    return result;
}

g3::LogMessageMover LogMerger::_addRepeats(g3::LogMessageMover &log, int repeats)
{
    // if no repeats, do nothing
    if (repeats == 0) {
        return log;
    }

    std::string writable_msg = log.get().write();

    // remove newline from end of message
    if (writable_msg.back() == '\n')
    {
        writable_msg.pop_back();
    }

    if (repeats > 1)
    {
        writable_msg += " (" + std::to_string(repeats) + " repeats)";
    } else {
        writable_msg += " (1 repeat)";
    }

    return log;
}

void LogMerger::pastime()
{
    passed_time += LOG_MERGE_DURATION;
}
