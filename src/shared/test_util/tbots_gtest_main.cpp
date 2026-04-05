#include "shared/test_util/tbots_gtest_main.h"

#include <fenv.h>

#include <boost/program_options.hpp>

#include "proto/parameters.pb.h"
#include "software/logger/logger.h"

bool TbotsGtestMain::help                = false;
bool TbotsGtestMain::enable_visualizer   = false;
bool TbotsGtestMain::run_sim_in_realtime = false;
bool TbotsGtestMain::stop_ai_on_start    = false;
std::string TbotsGtestMain::runtime_dir  = "/tmp/tbots/yellow_test";
double TbotsGtestMain::test_speed        = 1.0;

/**
 * Portable wrapper for feenableexcept.
 * Returns the previous set of enabled exceptions, or -1 on failure.
 */
int enable_fp_exceptions(unsigned int excepts)
{
#if defined(__linux__) && defined(__GNUC__)
    return feenableexcept(excepts);

#elif defined(__APPLE__)
    fenv_t fenv;
    if (fegetenv(&fenv) != 0)
    {
        return -1;
    }
    unsigned int old_excepts = (unsigned int)(fenv.__fpcr & FE_ALL_EXCEPT);

    // On ARM64, setting bits in FPCR enables the trap
    fenv.__fpcr |= (excepts & FE_ALL_EXCEPT);

    return (fesetenv(&fenv) == 0) ? (int)old_excepts : -1;

#else
    // Unsupported platform
    return -1;
#endif
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    // Crash on invalid operations like division by zero and floating-point overflow
    if (enable_fp_exceptions(FE_INVALID | FE_OVERFLOW) < 0)
    {
        std::cerr << "Warning: Could not enable floating-point exceptions." << std::endl;
    }

    // Crash on invalid operations like division by zero and floating-point overflow
    if (enable_fp_exceptions(FE_INVALID | FE_OVERFLOW) < 0)
    {
        std::cerr << "Warning: Could not enable floating-point exceptions." << std::endl;
    }

    boost::program_options::options_description desc{"Options"};

    // TODO #(2510) Remove this once we port over to simulated pytests entirely
    desc.add_options()("help,h",
                       boost::program_options::bool_switch(&TbotsGtestMain::help),
                       "Help screen");
    desc.add_options()(
        "enable_visualizer",
        boost::program_options::bool_switch(&TbotsGtestMain::enable_visualizer),
        "Displays simulated test on visualizer");
    desc.add_options()(
        "run_sim_in_realtime",
        boost::program_options::bool_switch(&TbotsGtestMain::run_sim_in_realtime),
        "Runs simulation in realtime, useful to look at simulated C++ tests in "
        "thunderscope without running ThreadedFullsystemGUI");
    desc.add_options()(
        "stop_ai_on_start",
        boost::program_options::bool_switch(&TbotsGtestMain::stop_ai_on_start),
        "If enable_visualizer is true, then stops the AI when the test starts");
    desc.add_options()(
        "runtime_dir",
        boost::program_options::value<std::string>(&TbotsGtestMain::runtime_dir),
        "The directory to output logs to. Absolute paths are recommended as the "
        "working directory is inside the bazel-out directory.");
    desc.add_options()(
        "test_speed", boost::program_options::value<double>(&TbotsGtestMain::test_speed),
        "The speed adjustment factor. Values in the range [0.1, 1) will play the"
        "test slower than realtime, and values in the range (1, 10] will play the"
        "test faster than realtime i.e. 0.1 would be 10X slower than realtime, "
        "and 10 would be 10X faster than realtime. Default value is 1.");

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (!TbotsGtestMain::help)
    {
        LoggerSingleton::initializeLogger(TbotsGtestMain::runtime_dir, nullptr);

        if (TbotsGtestMain::enable_visualizer || TbotsGtestMain::run_sim_in_realtime)
        {
            // disable floating point errors when using visualizer due to potential
            // floating point errors in QT
            // TODO #(2510) Remove this once we port over to simulated pytests entirely
            // fedisableexcept(FE_INVALID | FE_OVERFLOW);
        }
        return RUN_ALL_TESTS();
    }
    else
    {
        std::cout << desc << std::endl;
        return 0;
    }
}
