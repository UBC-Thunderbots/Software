#pragma once
#include <iostream>
#include <g3log/logmessage.hpp>

/**
 * This class acts a custom sink for g3log
 */
class ColouredCoutSink
{
public:
    // Linux xterm foreground color
    // http://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
    enum FG_Colour {
        YELLOW = 33,
        RED = 31,
        GREEN = 32,
        WHITE = 37
    };
    /**
     * This function is called on every call to LOG(). Displays coloured log messages in the terminal
     *
     * @params log_entry the Message received on a LOG() call
     */
    void displayColouredLog(g3::LogMessageMover log_entry);

private:
    /**
     * Gets the Colour associated with this log level
     *
     * @param level the level of the log message
     * @return the foreground colour associated with the given log level
     */
    FG_Colour getColour(const LEVELS level);

    /**
     * Sets the Colour of terminal messages to the default, white
     */
    void resetColour();
};
