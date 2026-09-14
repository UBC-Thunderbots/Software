#include <fenv.h>
#include <gtest/gtest.h>

#include "software/logger/logger.h"

std::string runtime_dir = "/tmp/tbots/yellow_test";

/**
 * Portable wrapper for feenableexcept. Use to specify which floating-point
 * exceptions should crash the program when they occur.
 *
 * @note On MacOS ARM, there are no floating-point exception traps, so tests may
 * pass on MacOS that wouldn't pass on other platforms if floating-point
 * exceptions occur.
 *
 * @param excepts A bitmask of floating-point exceptions to be enabled.
 * @return True on success, false on failure to set floating-point exceptions.
 */
bool enable_fp_exceptions(unsigned int excepts)
{
#if defined(__linux__) && defined(__GNUC__)
    feenableexcept(excepts);
    return true;
#else
    // Unsupported platform
    return false;
#endif
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    // Crash on invalid operations like sqrt of negative and floating-point overflow
    if (!enable_fp_exceptions(FE_INVALID | FE_OVERFLOW))
    {
        std::cerr << "Warning: Could not enable floating-point exceptions." << std::endl;
    }

    LoggerSingleton::initializeLogger(runtime_dir, nullptr);

    return RUN_ALL_TESTS();
}
