#pragma once
#include <zlib.h>

#include <optional>
#include <string>

class GenericLogger
{
public:
    static std::optional<GenericLogger> createLogger(const std::string& log_dir, const std::string& log_file_name);

    void close();

    ~GenericLogger();

    GenericLogger(GenericLogger&& other) noexcept;

    // 2. Move Assignment: "Steal" the handle during = assignment
    GenericLogger& operator=(GenericLogger&& other) noexcept;

    // 3. Explicitly delete Copying to prevent accidental double-frees
    GenericLogger(const GenericLogger&) = delete;
    GenericLogger& operator=(const GenericLogger&) = delete;

    // GenericLogger(const GenericLogger&) = delete;
    // GenericLogger& operator=(const GenericLogger&) = delete;
    //
    // GenericLogger(GenericLogger&& other) noexcept :
    //     log_gz_file_(other.log_gz_file_),
    //     log_file_path_(std::move(other.log_file_path_)) {
    //     other.log_gz_file_ = nullptr;
    // }
    //
    // GenericLogger& operator=(GenericLogger&& other) noexcept {
    //     if (this != &other) {
    //         close();
    //         log_gz_file_ = other.log_gz_file_;
    //         log_file_path_ = std::move(other.log_file_path_);
    //         other.log_gz_file_ = nullptr;
    //     }
    //     return *this;
    // }

    GenericLogger(gzFile log_gz_file, const std::string& log_file_path);

    bool log(const std::string& message) const;

    unsigned int getLogFileSize() const;

    std::string getLogFileName() const;

private:

    gzFile log_gz_file_;
    std::string log_file_path_;

};