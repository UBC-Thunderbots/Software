#include "software/logger/log_merger.h"

#include <gtest/gtest.h>

// helper function to create a LogMessage from a string
g3::LogMessage createLog(std::string msg)
{
    g3::LogMessage log = g3::LogMessage("file", 1, "function", WARNING);
    log.write()        = msg;
    return log;
}

// check if the messages stored in two logs are equal
void checkLogsEqual(g3::LogMessage m1, g3::LogMessage m2)
{
    EXPECT_EQ(m1.message(), m2.message());
}

TEST(LogMergerTests, add_repeats)
{
    LogMerger merger = LogMerger();

    g3::LogMessage log     = createLog("Logging is fun");
    g3::LogMessage repeats = createLog("Logging is fun (5 repeats)");
    merger._addRepeats(log, 5);

    checkLogsEqual(repeats, log);
}

TEST(LogMergerTests, log_one)
{
    LogMerger merger = LogMerger();

    g3::LogMessage msg = createLog("Just one log!");

    std::list<g3::LogMessage> logs = merger.log(msg);
    EXPECT_EQ(1, logs.size());
    checkLogsEqual(msg, logs.front());
}

TEST(LogMergerTests, log_two)
{
    LogMerger merger    = LogMerger();
    g3::LogMessage msg1 = createLog("Two logs now!");
    g3::LogMessage msg2 = createLog("Two logs now!");
    g3::LogMessage msg3 = createLog("Different message");

    std::list<g3::LogMessage> log1 = merger.log(msg1);
    EXPECT_EQ(1, log1.size());
    std::list<g3::LogMessage> log2 = merger.log(msg2);
    EXPECT_EQ(0, log2.size());

    merger.pastime();

    std::list<g3::LogMessage> log3 = merger.log(msg3);
    EXPECT_EQ(2, log3.size());

    g3::LogMessage repeat = createLog("Two logs now! (1 repeat)");
    checkLogsEqual(repeat, log3.front());
    log3.pop_front();
    checkLogsEqual(msg3, log3.front());
}

TEST(LogMergerTests, log_repeats)
{
    LogMerger merger = LogMerger();

    for (int i = 0; i < 50; i++)
    {
        g3::LogMessage msg = createLog("Lots of logs!");
        merger.log(msg);
    }
    merger.pastime();

    g3::LogMessage msg2           = createLog("New message");
    std::list<g3::LogMessage> log = merger.log(msg2);
    EXPECT_EQ(2, log.size());
    g3::LogMessage repeats = createLog("Lots of logs! (49 repeats)");
    checkLogsEqual(repeats, log.front());
    log.pop_front();
    checkLogsEqual(msg2, log.front());
}

TEST(LogMergerTests, log_different_repeats)
{
    LogMerger merger = LogMerger();

    for (int i = 0; i < 50; i++)
    {
        g3::LogMessage msg1 = createLog("First message");
        g3::LogMessage msg2 = createLog("Second message");
        merger.log(msg1);
        merger.log(msg2);
    }

    merger.pastime();

    g3::LogMessage newMsg         = createLog("New message");
    std::list<g3::LogMessage> log = merger.log(newMsg);
    EXPECT_EQ(3, log.size());

    g3::LogMessage repeats1 = createLog("First message (49 repeats)");
    g3::LogMessage repeats2 = createLog("Second message (49 repeats)");
    checkLogsEqual(repeats1, log.front());
    log.pop_front();
    checkLogsEqual(repeats2, log.front());
    log.pop_front();
    checkLogsEqual(newMsg, log.front());
}

TEST(LogMergerTests, log_one_in_middle)
{
    LogMerger merger = LogMerger();

    for (int i = 0; i < 10; i++)
    {
        g3::LogMessage msg1 = createLog("First message");
        merger.log(msg1);
    }

    g3::LogMessage msg2            = createLog("Middle");
    std::list<g3::LogMessage> log1 = merger.log(msg2);
    EXPECT_EQ(1, log1.size());
    checkLogsEqual(msg2, log1.front());

    merger.pastime();

    g3::LogMessage msg3            = createLog("First message");
    std::list<g3::LogMessage> log2 = merger.log(msg3);
    EXPECT_EQ(2, log2.size());

    g3::LogMessage repeats = createLog("First message (9 repeats)");
    checkLogsEqual(repeats, log2.front());
    log2.pop_front();
    checkLogsEqual(msg3, log2.front());
}

TEST(LogMergerTests, log_order)
{
    LogMerger merger = LogMerger();
    std::list<g3::LogMessage> logs;

    g3::LogMessage msg1 = createLog("First message");
    for (int i = 0; i < 5; i++) {
        logs = merger.log(msg1);
    }

    g3::LogMessage msg2 = createLog("Second message");
    for (int i = 0; i < 3; i++) {
        logs = merger.log(msg2);
    }

    g3::LogMessage msg3 = createLog("Third message");
    merger.log(msg3);

    // Interleaving msg1 and msg2
    for (int i = 0; i < 2; i++) {
        logs = merger.log(msg2);
    }

    for (int i = 0; i < 5; i++) {
        logs = merger.log(msg1);
    }

    merger.pastime();

    g3::LogMessage different_message = createLog("different message");
    logs = merger.log(different_message);

    EXPECT_EQ(3, logs.size());
    g3::LogMessage repeat_msg1 = createLog("First message (9 repeats)");
    logs.pop_front();
    g3::LogMessage repeat_msg2 = createLog("Second message (4 repeats)");
    logs.pop_front();
    checkLogsEqual(different_message, logs.front());

}