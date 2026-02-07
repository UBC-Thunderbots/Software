#include "software/logger/csv_sink.h"

#if __cplusplus > 201703L
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

CSVSink::CSVSink(const std::string& log_directory) : log_directory(log_directory) {}

void CSVSink::appendToFile(g3::LogMessageMover log_entry)
{
    if (log_entry.get()._level.value == CSV.value)
    {
        std::string msg = log_entry.get()._message;
        size_t pos      = msg.find(file_ext) + file_ext.length();

        if (pos != std::string::npos + file_ext.length())
        {
            std::string file_name = msg.substr(0, pos);
            std::string file_data = msg.substr(pos, msg.length());
            std::ofstream csv_file(log_directory + "/" + file_name,
                                   std::ios::out | std::ios_base::app);
            csv_file << file_data;
            csv_file.close();
        }
    }
}
