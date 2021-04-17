#pragma once
#include <g3log/logmessage.hpp>
#include "software/logger/custom_logging_levels.h"
#include <iostream>
#include <fstream>

/**
 * This class acts as a customer sink for g3log. In particular, it allows us to log to csv files.
 */
class CSVSink
{
public:
    /**
     * Creates a CSVSink that logs to the directory specified
     *
     * @param log_directory the directory to save files to
     */
    CSVSink(const std::string& log_directory);
    /**
     * This function is called on every call to LOG(CSV, filename). It appends to the specified file the message in log_entry.
     * Note for .csv files: columns are separated with "," and rows are separated with new line characters.
     *
     * @param log_entry the message received on a LOG() call
     *
     * @throws invalid_argument If the file name does not have .csv at the end
     */
    void appendToFile(g3::LogMessageMover log_entry);
private:
    std::string log_directory;
    const std::string file_ext = ".csv";
};