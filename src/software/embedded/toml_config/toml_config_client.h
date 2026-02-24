#pragma once

#include <mutex>
#include <string>
#include <toml++/toml.hpp>

#include "software/logger/logger.h"

/**
 * Client for reading and writing TOML configuration files.
 * Handles loading, reading, and writing robot configuration values.
 */
class TomlConfigClient
{
   public:
    /**
     * Creates a TOML config client that reads/writes to the specified file path.
     * If the file doesn't exist, it will be created with default values.
     *
     * @param config_file_path Path to the TOML configuration file
     */
    explicit TomlConfigClient(const std::string& config_file_path);

    virtual ~TomlConfigClient();

    /**
     * Gets the value corresponding to the key as a string.
     * Returns empty string if key doesn't exist.
     *
     * @param key The configuration key (e.g., "/robot_id")
     * @return The value as a string, or empty string if not found
     */
    std::string get(const std::string& key);

    /**
     * Sets a key-value pair in the TOML file.
     * The file is written immediately (synchronously).
     *
     * @param key The configuration key (e.g., "/robot_id")
     * @param value The value to set
     */
    void set(const std::string& key, const std::string& value);

    /**
     * Sets a key-value pair but doesn't write to disk immediately.
     * Call flush() to write all pending changes.
     *
     * @param key The configuration key
     * @param value The value to set
     */
    void setNoCommit(const std::string& key, const std::string& value);

    /**
     * Writes all pending changes to the TOML file.
     */
    void flush();

   private:
    std::string config_file_path_;
    toml::table config_table_;
    std::mutex config_mutex_;
    bool has_pending_changes_;

    /**
     * Loads the TOML file from disk into memory.
     * Creates a default file with default values if it doesn't exist.
     */
    void loadConfig();

    /**
     * Writes the current config_table_ to the TOML file.
     */
    void writeConfig();
};
