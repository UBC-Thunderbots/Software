
#include "generic_logger.h"

#include <cstring>
#include <iostream>

#include "compat_flags.h"

std::optional<GenericLogger> GenericLogger::createLogger(const std::string& log_dir, const std::string& log_file_name)
{
    fs::create_directories(log_dir);
    std::string log_file_path = log_dir + log_file_name;

    const gzFile gz_file = gzopen(log_file_path.c_str(), "wb");
    if (!gz_file)
    {
        std::cerr << "Failed to open gzip log file: " << log_file_path
                      << " Error: " + std::string(strerror(errno)) << std::endl;
        return std::nullopt;
    }

    return GenericLogger(gz_file, log_file_path);
}

GenericLogger::GenericLogger(gzFile log_gz_file, const std::string& log_file_path)
{
    log_gz_file_ = log_gz_file;
    log_file_path_ = log_file_path;
}

GenericLogger::GenericLogger(GenericLogger&& other) noexcept
    : log_gz_file_(other.log_gz_file_),
      log_file_path_(std::move(other.log_file_path_))
{
    other.log_gz_file_ = nullptr;
}

// Move Assignment
GenericLogger& GenericLogger::operator=(GenericLogger&& other) noexcept {
    if (this != &other) {
        close(); // Close our own current file before taking the new one
        log_gz_file_ = other.log_gz_file_;
        log_file_path_ = std::move(other.log_file_path_);
        other.log_gz_file_ = nullptr; // <--- Stop the other object from closing it
    }
    return *this;
}

GenericLogger::~GenericLogger()
{
    close();
}

void GenericLogger::close()
{
    if (log_gz_file_ != nullptr) {
        int result = gzclose(log_gz_file_);

        if (result != Z_OK)
        {
            std::cerr << "Failed to close log file: " << log_file_path_
                          << " with error " << result << std::endl;
        }
        log_gz_file_ = nullptr;
    }
}

bool GenericLogger::log(const std::string& message) const
{
    int num_bytes_written = gzwrite(log_gz_file_, message.data(), message.size());
    return num_bytes_written == static_cast<int>(message.size());
}

std::string GenericLogger::getLogFileName() const
{
    return log_file_path_;
}

unsigned int GenericLogger::getLogFileSize() const
{
    return gzoffset(log_gz_file_);
}


