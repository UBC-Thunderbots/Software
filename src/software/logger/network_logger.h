#pragma once

#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>
#include <g3log/logmessage.hpp>
#include <g3log/logworker.hpp>

#include "software/logger/coloured_cout_sink.h"
#include "software/logger/network_sink.h"

static const std::string CSV_PATH = "/tmp";

/**
 * This class acts as a Singleton that's responsible for initializing the logger.
 */
class NetworkLoggerSingleton
{
   public:
    /**
     * Initializes a g3log logger for the calling program. This should only be
     * called once at the start of a program.
     */
    static void initializeLogger(int channel, const std::string& interface, int robot_id,
                                 bool enable_log_merging)
    {
        static std::shared_ptr<NetworkLoggerSingleton> s(
            new NetworkLoggerSingleton(channel, interface, robot_id, enable_log_merging));
    }


   private:
    NetworkLoggerSingleton(int channel, const std::string& interface, int robot_id,
                           bool enable_log_merging)
    {
        logWorker = g3::LogWorker::createLogWorker();

        auto network_log_sink_handle =
            logWorker->addSink(std::make_unique<struct NetworkSink>(
                                   channel, interface, robot_id, enable_log_merging),
                               &NetworkSink::sendToNetwork);

        // Sink for outputting logs to the terminal
        auto colour_cout_sink_handle =
            logWorker->addSink(std::make_unique<ColouredCoutSink>(true),
                               &ColouredCoutSink::displayColouredLog);

        auto csv_sink_handle = logWorker->addSink(std::make_unique<CSVSink>(CSV_PATH),
                                                  &CSVSink::appendToFile);

        // Sink for PlotJuggler plotting
        auto plotjuggler_handle = logWorker->addSink(std::make_unique<PlotJugglerSink>(),
                                                     &PlotJugglerSink::sendToPlotJuggler);

        g3::initializeLogging(logWorker.get());
    }

    std::unique_ptr<g3::LogWorker> logWorker;
};
