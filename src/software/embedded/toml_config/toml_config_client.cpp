#include "software/embedded/toml_config/toml_config_client.h"

#include <filesystem>
#include <fstream>

TomlConfigClient::TomlConfigClient(const std::string& config_file_path)
    : config_file_path_(config_file_path), has_pending_changes_(false)
{
    loadConfig();
}

TomlConfigClient::~TomlConfigClient()
{
    // Flush any pending changes on destruction
    if (has_pending_changes_)
    {
        flush();
    }
}

void TomlConfigClient::loadConfig()
{
    std::lock_guard<std::mutex> lock(config_mutex_);

    // Create directory if it doesn't exist
    std::filesystem::path config_path(config_file_path_);
    std::filesystem::path config_dir = config_path.parent_path();
    if (!config_dir.empty() && !std::filesystem::exists(config_dir))
    {
        std::filesystem::create_directories(config_dir);
        LOG(INFO) << "Created TOML config directory: " << config_dir;
    }

    // Check if file exists
    if (!std::filesystem::exists(config_file_path_))
    {
        LOG(INFO) << "TOML config file does not exist, creating default: "
                  << config_file_path_;
        // Create table with default values
        config_table_ = toml::table{
            {"robot_id", "1"},
            {"channel_id", "0"},
            {"network_interface", "tbotswifi5"},
            {"kick_constant", "0"},
            {"kick_coeff", "0.0"},
            {"chip_pulse_width", "0"},
            {"battery_voltage", "0.0"},
            {"current_draw", "0.0"},
            {"cap_voltage", "0.0"},
        };
        writeConfig();
        return;
    }

    try
    {
        config_table_ = toml::parse_file(config_file_path_);
        LOG(INFO) << "Successfully loaded TOML config from: " << config_file_path_;
    }
    catch (const toml::parse_error& err)
    {
        LOG(WARNING) << "Failed to parse TOML config file: " << config_file_path_
                     << ". Error: " << err.description();
        // Create empty table on parse error
        config_table_ = toml::table{};
    }
}

void TomlConfigClient::writeConfig()
{
    std::ofstream file(config_file_path_);
    if (!file.is_open())
    {
        LOG(WARNING) << "Failed to open TOML config file for writing: "
                     << config_file_path_;
        return;
    }

    file << config_table_;
    file.close();
    has_pending_changes_ = false;
}

std::string TomlConfigClient::normalizeKey(const std::string& key)
{
    // Remove leading slash if present
    if (!key.empty() && key[0] == '/')
    {
        return key.substr(1);
    }
    return key;
}

std::string TomlConfigClient::get(const std::string& key)
{
    std::lock_guard<std::mutex> lock(config_mutex_);

    std::string normalized_key = normalizeKey(key);

    try
    {
        auto node = config_table_[normalized_key];
        if (node.is_string())
        {
            return node.as_string()->get();
        }
        else if (node.is_integer())
        {
            return std::to_string(node.as_integer()->get());
        }
        else if (node.is_floating_point())
        {
            return std::to_string(node.as_floating_point()->get());
        }
        else if (node.is_boolean())
        {
            return node.as_boolean()->get() ? "1" : "0";
        }
        else
        {
            LOG(WARNING) << "TOML key '" << key << "' exists but is not a supported type";
            return "";
        }
    }
    catch (const std::exception& e)
    {
        LOG(WARNING) << "TOML key '" << key << "' not found: " << e.what();
        return "";
    }
}

void TomlConfigClient::set(const std::string& key, const std::string& value)
{
    std::lock_guard<std::mutex> lock(config_mutex_);

    std::string normalized_key = normalizeKey(key);
    config_table_.insert_or_assign(normalized_key, value);
    has_pending_changes_ = true;
    writeConfig();
}

void TomlConfigClient::setNoCommit(const std::string& key, const std::string& value)
{
    std::lock_guard<std::mutex> lock(config_mutex_);

    std::string normalized_key = normalizeKey(key);
    config_table_.insert_or_assign(normalized_key, value);
    has_pending_changes_ = true;
}

void TomlConfigClient::flush()
{
    std::lock_guard<std::mutex> lock(config_mutex_);

    if (has_pending_changes_)
    {
        writeConfig();
    }
}
