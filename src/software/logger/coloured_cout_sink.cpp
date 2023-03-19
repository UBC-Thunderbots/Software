#include "software/logger/coloured_cout_sink.h"

#include "software/logger/custom_logging_levels.h"

ColouredCoutSink::ColouredCoutSink(bool print_detailed)
    : print_detailed(print_detailed), merger(LogMerger())
{
}

std::string ColouredCoutSink::colourToString(const FG_Colour colour)
{
    switch (colour)
    {
        case (FG_Colour::YELLOW):
            return "33";
        case (FG_Colour::RED):
            return "31";
        case (FG_Colour::GREEN):
            return "32";
        case (FG_Colour::WHITE):
            return "37";
        default:
            return "";
    }
}

FG_Colour ColouredCoutSink::getColour(const LEVELS level)
{
    if (level.value == WARNING.value)
    {
        return FG_Colour::YELLOW;
    }
    if (level.value == G3LOG_DEBUG.value)
    {
        return FG_Colour::GREEN;
    }
    if (g3::internal::wasFatal(level))
    {
        return FG_Colour::RED;
    }

    return FG_Colour::WHITE;
}

void ColouredCoutSink::resetColour()
{
    std::ostringstream oss;
    oss << "\033[" << colourToString(FG_Colour::WHITE) << "m"
        << " "
        << "\033[m";
    std::cout << oss.str();
}

void ColouredCoutSink::displayColouredLog(g3::LogMessageMover log_entry)
{
    auto level  = log_entry.get()._level;
    auto colour = colourToString(getColour(level));

    if (level.value == VISUALIZE.value || level.value == CSV.value)
    {
        // Don't log anything that calls LOG(VISUALIZE) and LOG(CSV)
        return;
    }

    std::ostringstream oss;
    if (print_detailed)
    {
        // detailed messages have timing data, do not merge
        oss << "\033[" << colour << "m" << log_entry.get().toString() << "\033[m";
    }
    else
    {
        std::list<std::string> logs = merger.log(log_entry.get().message());
        for (std::string msg : logs)
        {
            oss << "\033[" << colour << "m" << msg << "\n\033[m";
        }
    }
    std::cout << oss.str() << std::flush;
    resetColour();
}
