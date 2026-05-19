#include <fenv.h>
#include <gtest/gtest.h>

#include "software/logger/logger.h"

std::string runtime_dir = "/tmp/tbots/yellow_test";

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    feenableexcept(FE_INVALID | FE_OVERFLOW);

    LoggerSingleton::initializeLogger(runtime_dir, nullptr);

    return RUN_ALL_TESTS();
}
