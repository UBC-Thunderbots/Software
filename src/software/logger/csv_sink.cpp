#include "software/logger/csv_sink.h"

#if __cplusplus > 201703L
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

CSVSink::CSVSink(const std::string& log_directory) : log_directory(log_directory) {}

void CSVSink::writeToFile(g3::LogMessageMover log_entry)
{
    if (log_entry.get()._level.value == CSV.value ||
        log_entry.get()._level.value == CSV_OVERWRITE.value)
    {
        std::string msg = log_entry.get()._message;
        size_t pos      = msg.find(file_ext) + file_ext.length();

        if (pos != std::string::npos + file_ext.length())
        {
            std::string file_name = msg.substr(0, pos);
            std::string file_data = msg.substr(pos, msg.length());

            std::ios_base::openmode open_mode = std::ios::out;
            if (log_entry.get()._level.value == CSV.value)
            {
                // Append mode
                open_mode |= std::ios_base::app;
            }
            else
            {
                // Overwrite mode
                open_mode |= std::ios_base::trunc;
            }

            std::ofstream csv_file(log_directory + "/" + file_name, open_mode);
            csv_file << file_data;
            csv_file.close();
        }
    }
}
