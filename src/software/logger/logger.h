#pragma once

#include <g3log/g3log.hpp>
#include <g3log/logworker.hpp>

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
        g3::initializeLogging(logWorker.get());
    }

    std::unique_ptr<g3::LogWorker> logWorker;
};
