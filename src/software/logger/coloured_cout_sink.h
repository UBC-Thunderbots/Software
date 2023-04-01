#pragma once
#include <g3log/logmessage.hpp>
#include <iostream>

#include "software/util/make_enum/make_enum.h"
#include "software/logger/log_merger.h"

MAKE_ENUM(FG_Colour, YELLOW, RED, GREEN, WHITE);
/**
 * This class acts a custom sink for g3log
 */
class ColouredCoutSink
{
   public:
    /**
     * creates a ColouredCoutSink which prints logs in different colours based on their
     * level
     * @param print_detailed If true, prints the log message as well as other details
     * (level, file line, etc). If false, only prints the message
     */
    ColouredCoutSink(bool print_detailed);
    /**
     * This is a helper function for mapping the FG_Colour enum to its relative
     * Linux xterm foreground color
     * http://stackoverflow.com/questions/2616906/how-do-i-output-coloured-text-to-a-linux-terminal
     *
     * @param colour the colour to be converted
     * @return the string representation of the FG_Colour
     */
    static std::string colourToString(const FG_Colour colour);
    /**
     * This function is called on every call to LOG(). Displays coloured log messages in
     * the terminal. Merges repeated messages to display later
     *
     * @params log_entry the message received on a LOG() call
     */
    void displayColouredLog(g3::LogMessageMover log_entry);

    /**
     * Displays a single coloured log message in the terminal, without merging.
     * 
     * @params log the LogMessage to output
    */
    void displaySingleLog(g3::LogMessage &log);

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

    /*
     * Determines whether log details (file, line, level, etc) are printed alongside the
     * log message
     */
    bool print_detailed;

    LogMerger log_merger;
};
