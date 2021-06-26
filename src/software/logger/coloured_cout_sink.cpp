#include "software/logger/coloured_cout_sink.h"

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

    std::ostringstream oss;
    oss << "\033[" << colour << "m" << log_entry.get().toString() << "\033[m";
    std::cout << oss.str() << std::flush;
    resetColour();
}
