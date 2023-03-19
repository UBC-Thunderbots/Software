#include "software/logger/log_merger.h"

LogMerger::LogMerger() : passed_time(std::chrono::seconds(0)) {}

std::list<std::string> LogMerger::log(std::string msg)
{
    std::chrono::_V2::system_clock::time_point current_time =
        std::chrono::system_clock::now();
    // add passed time from testing
    current_time += passed_time;
    std::list<std::string> messages = getOldMessages(current_time);

    if (repeat_map.count(msg))
    {
        // msg is in the repeat map, add a repeat and return the old messages
        repeat_map[msg]++;
        return messages;
    }

    // msg is not in the repeat map, add it to the map and list and log it
    repeat_map[msg] = 0;
    message_list.push_back(Message(msg, current_time));

    messages.push_front(msg);
    return messages;
}

std::list<std::string> LogMerger::getOldMessages(
    std::chrono::_V2::system_clock::time_point current_time)
{
    std::list<std::string> result;
    while (message_list.size() > 0)
    {
        Message currentMessage = message_list.front();
        if (current_time - LOG_MERGE_DURATION >= currentMessage.timestamp &&
            repeat_map[currentMessage.message] > 0)
        {
            // old message w/ at least 1 repeat
            std::string currentString = currentMessage.message;
            result.push_back(addRepeats(currentString, repeat_map[currentString]));
            repeat_map.erase(currentString);
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

std::string LogMerger::addRepeats(std::string msg, int repeats)
{
    // remove newline from end of message
    if (msg.back() == '\n')
    {
        msg.pop_back();
    }

    if (repeats > 1)
    {
        msg += " (" + std::to_string(repeats) + " repeats)";
    }
    return msg;
}

void LogMerger::pastime()
{
    passed_time += LOG_MERGE_DURATION;
}