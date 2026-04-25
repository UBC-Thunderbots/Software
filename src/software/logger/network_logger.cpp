#include "software/logger/network_logger.h"

#include "software/logger/csv_sink.h"
#include "software/logger/plotjuggler_sink.h"

std::shared_ptr<NetworkLoggerSingleton> NetworkLoggerSingleton::instance;

NetworkLoggerSingleton::NetworkLoggerSingleton(RobotId robot_id, bool enable_log_merging)
{
    logWorker = g3::LogWorker::createLogWorker();

    network_sink_handle =
        logWorker->addSink(std::make_unique<NetworkSink>(robot_id, enable_log_merging),
                           &NetworkSink::sendToNetwork);

    logWorker->addSink(std::make_unique<ColouredCoutSink>(true),
                       &ColouredCoutSink::displayColouredLog);

    logWorker->addSink(std::make_unique<CSVSink>(CSV_PATH), &CSVSink::appendToFile);

    logWorker->addSink(std::make_unique<PlotJugglerSink>("tbotswifi5"),
                       &PlotJugglerSink::sendToPlotJuggler);

    g3::only_change_at_initialization::addLogLevel(CSV);
    g3::only_change_at_initialization::addLogLevel(PLOTJUGGLER);

    g3::initializeLogging(logWorker.get());
}

void NetworkLoggerSingleton::initializeLogger(RobotId robot_id, bool enable_log_merging)
{
    if (!instance)
    {
        instance = std::shared_ptr<NetworkLoggerSingleton>(
            new NetworkLoggerSingleton(robot_id, enable_log_merging));
    }
}

void NetworkLoggerSingleton::replaceUdpSender(
    std::shared_ptr<ThreadedProtoUdpSender<TbotsProto::RobotLog>> new_sender)
{
    if (!instance)
    {
        return;
    }
    instance->network_sink_handle->call(&NetworkSink::replaceUdpSender, new_sender);
}
