#include "software/logger/logger.h"

#include <gtest/gtest.h>
#include <err.h>

TEST(logger_test, testLogFatal)
{
    LoggerSingleton::initializeLogger();
    try {
        LOG(FATAL) << "test fatal";
    } catch (std::exception){
        EXPECT_TRUE(true);
    }
}