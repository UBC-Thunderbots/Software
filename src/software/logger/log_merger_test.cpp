#include "software/logger/log_merger.h"

#include <gtest/gtest.h>

// helper function to create a LogMessageMover from a string
g3::LogMessageMover createDefaultMover(std::string &msg) {
    g3::LogMessage log = g3::LogMessage("file", 1, "function", WARNING);
    log.write() = msg;
    return g3::LogMessageMover(std::move(log));
}

// check if the messages stored in two log mergers are equal
void checkMoversEqual(g3::LogMessageMover m1, g3::LogMessageMover m2) {
    EXPECT_EQ(m1.get().message(), m2.get().message());
}

TEST(LogMergerTests, add_repeats) {
    LogMerger merger = LogMerger();

    g3::LogMessageMover log = createDefaultMover("Logging is fun");
    g3::LogMessageMover repeats = createDefaultMover("Logging is fun (5 repeats)");
    merger._addRepeats(log, 5);

    checkMoversEqual(repeats, log);
}

TEST(LogMergerTests, log_one)
{
    LogMerger merger = LogMerger();

    g3::LogMessageMover msg = createDefaultMover("Just one log!");

    std::list<g3::LogMessageMover> logs = merger.log(msg);
    EXPECT_EQ(1, logs.size());
    checkMoversEqual(msg, logs.front());
}

TEST(LogMergerTests, log_two)
{
    LogMerger merger = LogMerger();
    g3::LogMessageMover msg1 = createDefaultMover("Two logs now!");
    g3::LogMessageMover msg2 = createDefaultMover("Two logs now!");
    g3::LogMessageMover msg3 = createDefaultMover("Different message");

    std::list<g3::LogMessageMover> log1 = merger.log(msg1);
    EXPECT_EQ(1, log1.size());
    std::list<g3::LogMessageMover> log2 = merger.log(msg2);
    EXPECT_EQ(0, log2.size());

    merger.pastime();

    std::list<g3::LogMessageMover> log3 = merger.log(msg3);
    EXPECT_EQ(2, log3.size());

    g3::LogMessageMover repeat = createDefaultMover("Two logs now! (1 repeat)");
    checkMoversEqual(msg3, log3.front());
    log3.pop_front();
    checkMoversEqual(repeat, log3.front());
}

// TEST(LogMergerTests, log_repeats)
// {
//     LogMerger merger = LogMerger();
//     std::string msg  = "Lots of logs!";
//     std::string msg2 = "New message";
//     for (int i = 0; i < 50; i++)
//     {
//         merger.log(msg);
//     }
//     merger.pastime();

//     std::list<std::string> log = merger.log(msg2);
//     EXPECT_EQ(2, log.size());
//     EXPECT_EQ(msg2, log.front());
//     log.pop_front();
//     EXPECT_EQ(msg + " (49 repeats)", log.front());
// }

// TEST(LogMergerTests, log_different_repeats)
// {
//     LogMerger merger   = LogMerger();
//     std::string msg1   = "First message";
//     std::string msg2   = "Second message";
//     std::string newMsg = "New message";

//     for (int i = 0; i < 50; i++)
//     {
//         merger.log(msg1);
//         merger.log(msg2);
//     }
//     merger.pastime();

//     std::list<std::string> log = merger.log(newMsg);
//     EXPECT_EQ(3, log.size());
//     EXPECT_EQ(newMsg, log.front());
//     log.pop_front();
//     EXPECT_EQ(msg1 + " (49 repeats)", log.front());
//     log.pop_front();
//     EXPECT_EQ(msg2 + " (49 repeats)", log.front());
// }

// TEST(LogMergerTests, log_one_in_middle)
// {
//     LogMerger merger = LogMerger();
//     std::string msg1 = "First message";
//     std::string msg2 = "Second message";

//     for (int i = 0; i < 10; i++)
//     {
//         merger.log(msg1);
//     }
//     std::list<std::string> log1 = merger.log(msg2);
//     EXPECT_EQ(1, log1.size());
//     EXPECT_EQ(msg2, log1.front());
//     merger.pastime();

//     std::list<std::string> log2 = merger.log(msg1);
//     EXPECT_EQ(2, log2.size());
//     EXPECT_EQ(msg1, log2.front());
//     log2.pop_front();
//     EXPECT_EQ(msg1 + " (9 repeats)", log2.front());
// }
