#include "software/logger/log_merger.h"

LogMerger::LogMerger() : passed_time(std::chrono::seconds(0)) {}

std::list<g3::LogMessage> LogMerger::log(g3::LogMessage &log)
{
    std::string msg = log.message();

    std::chrono::_V2::system_clock::time_point current_time =
        std::chrono::system_clock::now();
    // add passed time from testing
    current_time += passed_time;
    std::list<g3::LogMessage> messages_to_log = _getOldMessages(current_time);

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

std::list<g3::LogMessage> LogMerger::_getOldMessages(
    std::chrono::_V2::system_clock::time_point current_time)
{
    std::list<g3::LogMessage> result;
    while (message_list.size() > 0)
    {
        Message current_message = message_list.front();
        if (current_time - LOG_MERGE_DURATION >= current_message.timestamp)
        {
            // old, if it has repeats, add repeats to the message and add it to the list
            int repeats = repeat_map[current_message.msg];
            if (repeats > 0)
            {
                g3::LogMessage log = current_message.log;
                result.push_back(_addRepeats(log, repeats));
            }
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

g3::LogMessage LogMerger::_addRepeats(g3::LogMessage &log, int repeats)
{
    // if no repeats, do nothing
    if (repeats == 0)
    {
        return log;
    }

    // remove newline from end of message
    if (log.write().back() == '\n')
    {
        log.write().pop_back();
    }

    if (repeats > 1)
    {
        log.write() += " (" + std::to_string(repeats) + " repeats)";
    }
    else
    {
        log.write() += " (1 repeat)";
    }

    return log;
}

void LogMerger::pastime()
{
    passed_time += LOG_MERGE_DURATION;
}
