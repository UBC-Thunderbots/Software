#pragma once

#include <g3sinks/LogRotate.h>
#include <g3sinks/LogRotateWithFilter.h>

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>
#include <g3log/logmessage.hpp>
#include <g3log/logworker.hpp>

#include "software/logger/coloured_cout_sink.h"
#include "software/logger/custom_logging_levels.h"

/**
 * This class acts as a Singleton that's responsible for initializing the logger.
 * We use a singleton rather than a generic function in this namespace because
 * the logWorker object must stay in scope, otherwise the logger is automatically
 * destroyed. So if an "init" function is used, the logWorker goes out of scope as
 * soon as the function returns, which destroys the logger right after creating it
 *
 * The Singleton class allows us to keep the logWorker in scope for the duration
 * of the program while still providing a single function to initialize the logger
 */
class LoggerSingleton
{
   public:
    /**
     * Initializes a g3log logger for the calling program. This should only be
     * called once at the start of a program.
     */
    static void initializeLogger()
    {
        static std::shared_ptr<LoggerSingleton> s(new LoggerSingleton());
    }


   private:
    LoggerSingleton()
    {
        logWorker = g3::LogWorker::createLogWorker();
        // Robot diagnostics logs are in
        // bazel-out/k8-fastbuild/bin/software/gui/robot_diagnostics/robot_diagnostics_main.runfiles/__main__/software/
        // Full system logs are in
        // bazel-out/k8-fastbuild/bin/software/full_system.runfiles/__main__/software/
        // Simulated test logs are in
        // bazel-out/k8-fastbuild/bin/software/simulated_tests/TEST_NAME.runfiles/__main__/software/
        // where TEST_NAME is the name of the simulated test

        // Sink for outputting logs to the terminal
        auto colour_cout_sink_handle = logWorker->addSink(
            std::make_unique<ColouredCoutSink>(), &ColouredCoutSink::displayColouredLog);
        // Sink for storing a file of all logs
        auto log_rotate_sink_handle = logWorker->addSink(
            std::make_unique<LogRotate>(log_name, "./software/"), &LogRotate::save);
        // Sink for storing a file of filtered logs
        auto filtered_log_rotate_sink_handle = logWorker->addSink(
            std::make_unique<LogRotateWithFilter>(
                std::make_unique<LogRotate>(log_name + filter_suffix, "./software/"),
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
