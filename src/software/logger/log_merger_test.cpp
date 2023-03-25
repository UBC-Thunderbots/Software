#include "software/logger/log_merger.h"

#include <gtest/gtest.h>

TEST(LogMergerTests, log_one)
{
    LogMerger merger = LogMerger();
    std::string msg  = "Just one log!";

    std::list<std::string> log1 = merger.log(msg);
    EXPECT_EQ(1, log1.size());
    EXPECT_EQ(msg, log1.front());
}

TEST(LogMergerTests, log_two)
{
    LogMerger merger = LogMerger();
    std::string msg  = "Two logs now!";

    std::list<std::string> log1 = merger.log(msg);
    std::list<std::string> log2 = merger.log(msg);
    EXPECT_EQ(0, log2.size());

    merger.pastime();

    std::list<std::string> log3 = merger.log(msg);
    EXPECT_EQ(2, log3.size());
    EXPECT_EQ(msg, log3.front());
    log3.pop_front();
    EXPECT_EQ(msg, log3.front());
}

TEST(LogMergerTests, log_repeats)
{
    LogMerger merger = LogMerger();
    std::string msg  = "Lots of logs!";
    std::string msg2 = "New message";
    for (int i = 0; i < 50; i++)
    {
        merger.log(msg);
    }
    merger.pastime();

    std::list<std::string> log = merger.log(msg2);
    EXPECT_EQ(2, log.size());
    EXPECT_EQ(msg2, log.front());
    log.pop_front();
    EXPECT_EQ(msg + " (49 repeats)", log.front());
}

TEST(LogMergerTests, log_different_repeats)
{
    LogMerger merger   = LogMerger();
    std::string msg1   = "First message";
    std::string msg2   = "Second message";
    std::string newMsg = "New message";

    for (int i = 0; i < 50; i++)
    {
        merger.log(msg1);
        merger.log(msg2);
    }
    merger.pastime();

    std::list<std::string> log = merger.log(newMsg);
    EXPECT_EQ(3, log.size());
    EXPECT_EQ(newMsg, log.front());
    log.pop_front();
    EXPECT_EQ(msg1 + " (49 repeats)", log.front());
    log.pop_front();
    EXPECT_EQ(msg2 + " (49 repeats)", log.front());
}

TEST(LogMergerTests, log_one_in_middle)
{
    LogMerger merger = LogMerger();
    std::string msg1 = "First message";
    std::string msg2 = "Second message";

    for (int i = 0; i < 10; i++)
    {
        merger.log(msg1);
    }
    std::list<std::string> log1 = merger.log(msg2);
    EXPECT_EQ(1, log1.size());
    EXPECT_EQ(msg2, log1.front());
    merger.pastime();

    std::list<std::string> log2 = merger.log(msg1);
    EXPECT_EQ(2, log2.size());
    EXPECT_EQ(msg1, log2.front());
    log2.pop_front();
    EXPECT_EQ(msg1 + " (9 repeats)", log2.front());
}
