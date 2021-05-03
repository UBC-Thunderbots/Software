#pragma once

#include <g3sinks/LogRotate.h>
#include <g3sinks/LogRotateWithFilter.h>

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>
#include <g3log/logmessage.hpp>
#include <g3log/logworker.hpp>

#include "software/logger/coloured_cout_sink.h"
#include "software/logger/csv_sink.h"
#include "software/logger/custom_logging_levels.h"

// This undefines LOG macro defined by g3log
#undef LOG

// These macros allows us to overload arguments.
// https://stackoverflow.com/questions/11761703/overloading-macro-on-number-of-arguments
#define LOG_SELECT(_1, _2, NAME, ...) NAME
#define LOG(...) LOG_SELECT(__VA_ARGS__, LOG_2, LOG_1)(__VA_ARGS__)

// Called when LOG() is called with 1 argument. This is a copy of g3log's LOG() macro
// Note: curly braces are not used as we need to pipe log messages to the logger
#define LOG_1(level)                                                                     \
    if (!g3::logLevel(level))                                                            \
    {                                                                                    \
    }                                                                                    \
    else                                                                                 \
        INTERNAL_LOG_MESSAGE(level).stream()

// Called when LOG() is called with 2 arguments
#define LOG_2(level, filename)                                                           \
    if (level != CSV)                                                                    \
    {                                                                                    \
    }                                                                                    \
    else                                                                                 \
        LOG_1(level) << filename                                                         \
/**                                                                                      \
 * This class acts as a Singleton that's responsible for initializing the logger.        \
 * We use a singleton rather than a generic function in this namespace because           \
 * the logWorker object must stay in scope, otherwise the logger is automatically        \
 * destroyed. So if an "init" function is used, the logWorker goes out of scope as       \
 * soon as the function returns, which destroys the logger right after creating it       \
 *                                                                                       \
 * The Singleton class allows us to keep the logWorker in scope for the duration         \
 * of the program while still providing a single function to initialize the logger       \
 */
class LoggerSingleton
{
   public:
    /**
     * Initializes a g3log logger for the calling program. This should only be
     * called once at the start of a program.
     */
    static void initializeLogger(const std::string& log_directory)
    {
        static std::shared_ptr<LoggerSingleton> s(new LoggerSingleton(log_directory));
    }


   private:
    LoggerSingleton(const std::string& log_directory)
    {
        logWorker = g3::LogWorker::createLogWorker();
        // Default locations
        // Full system: bazel-out/k8-fastbuild/bin/software/full_system.runfiles/__main__/
        // Robot diagnostics:
        // bazel-out/k8-fastbuild/bin/software/gui/robot_diagnostics/robot_diagnostics_main.runfiles/__main__/
        // Standalone Simulator:
        // bazel-out/k8-fastbuild/bin/software/simulation/standalone_simulator_main.runfiles/__main__/
        // Simulated
        // tests:bazel-out/k8-fastbuild/bin/software/simulated_tests/TEST_NAME.runfiles/__main__/
        //   where TEST_NAME is the name of the simulated test

        // Log locations can also be defined by setting the --logging_dir command line
        // arg. Note: log locations are defaulted to the bazel-out folder due to Bazel's
        // hermetic build principles

        auto csv_sink_handle = logWorker->addSink(
            std::make_unique<CSVSink>(log_directory), &CSVSink::appendToFile);
        // Sink for outputting logs to the terminal
        auto colour_cout_sink_handle = logWorker->addSink(
            std::make_unique<ColouredCoutSink>(), &ColouredCoutSink::displayColouredLog);
        // Sink for storing a file of all logs
        auto log_rotate_sink_handle = logWorker->addSink(
            std::make_unique<LogRotate>(log_name, log_directory), &LogRotate::save);
        // Sink for storing a file of filtered logs
        auto filtered_log_rotate_sink_handle = logWorker->addSink(
            std::make_unique<LogRotateWithFilter>(
                std::make_unique<LogRotate>(log_name + filter_suffix, log_directory),
                level_filter),
            &LogRotateWithFilter::save);

        g3::initializeLogging(logWorker.get());
    }

    // levels is this vector are filtered out of the filtered log rotate sink
    std::vector<LEVELS> level_filter = {DEBUG, INFO, ROBOT_STATUS};
    const std::string filter_suffix  = "_filtered";
    const std::string log_name       = "thunderbots";
    std::unique_ptr<g3::LogWorker> logWorker;
};
