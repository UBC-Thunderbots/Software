#pragma once
#include "software/networking/threaded_udp_sender.h"
#include <g3log/logmessage.hpp>
#include <string>

#include "proto/visualization.pb.h"
#include "software/logger/custom_logging_levels.h"


/**
 * This class acts as a custom sink for g3log. In particular, it allows us to send
 * values to be plotted in PlotJuggler over UDP.
 */
class PlotJugglerSink
{
   public:
    /**
     * Creates a PlotJugglerSink that sends udp packets to the PlotJuggler server
     */
    PlotJugglerSink();

    ~PlotJugglerSink() = default;

    /**
     * This function is called on every call to LOG(). It sends a JSON string to
     * PlotJuggler over the network.
     *
     * @param log_entry the message received on a LOG() call
     */
    void sendToPlotJuggler(g3::LogMessageMover log_entry);

   private:
    ThreadedUdpSender udp_sender;
};

/*
 * TODO
 *
 * @param os The output stream
 * @param message The message to serialize
 * @return The output stream containing
 */
std::ostream& operator<<(std::ostream& os,
                         const TbotsProto::PlotJugglerValue& plotjuggler_value);
