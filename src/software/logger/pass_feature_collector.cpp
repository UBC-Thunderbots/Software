
#include "pass_feature_collector.h"

#include <utility>
#include "compat_flags.h"
#include <zlib.h>

PassFeatureCollector::PassFeatureCollector(std::string log_dir, bool friendly_colour_yellow)
    : log_dir_(std::move(log_dir)),
      friendly_colour_yellow_(friendly_colour_yellow)
{
    log_path_ = log_dir + "/" + PASS_FEATURE_FILE;
    fs::create_directories(log_path_);
}

PassFeatureCollector::PassFeatureCollector(const std::string& log_path, bool friendly_colour_yellow)
{

}

void PassFeatureCollector::logFeaturesToFile(std::string log_entry)
{
    gzFile gz_file = gzopen(log_path_.c_str(), "wb");
    if (!gz_file)
    {
        std::cerr << "PassFeatureCollector: Failed to open gzip log file: " << log_path_
                  << " Error: " + std::string(strerror(errno)) << std::endl;
        return;
    }

    // Write the log entry to the file with the format:
    std::string log_entry =
        createLogEntry(proto_full_name, serialized_proto, receive_time_sec);
    gzwrite(gz_file, log_entry.c_str(), static_cast<unsigned>(log_entry.size()));

    int result = gzclose(gz_file);
    if (result != Z_OK)
    {
        std::cerr << "PassFeatureCollector: Failed to close log file: " << log_path_
                  << " with error " << result << std::endl;
    }
}

