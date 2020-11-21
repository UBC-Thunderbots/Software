#include "software/logger/coloured_cout_sink.h"

ColouredCoutSink::FG_Colour ColouredCoutSink::getColour(const LEVELS level)
{
    if (level.value == WARNING.value)
    {
        return YELLOW;
    }
    if (level.value == G3LOG_DEBUG.value)
    {
        return GREEN;
    }
    if (g3::internal::wasFatal(level))
    {
        return RED;
    }

    return WHITE;
}

void ColouredCoutSink::resetColour()
{
    std::ostringstream oss;
    oss << "\033[" << WHITE << "m"
        << " "
        << "\033[m";
    std::cout << oss.str();
}

void ColouredCoutSink::displayColouredLog(g3::LogMessageMover log_entry)
{
    auto level  = log_entry.get()._level;
    auto colour = getColour(level);

    std::ostringstream oss;
    oss << "\033[" << colour << "m" << log_entry.get().toString() << "\033[m"
        << std::endl;
    std::cout << oss.str();
    resetColour();
}
