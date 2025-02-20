#pragma once

#include <string>

#include "software/constants.h"
#include "yaml-cpp/yaml.h"

const std::string KEYS_TO_CHECK[] = {ROBOT_ID_REDIS_KEY,
                                     ROBOT_MULTICAST_CHANNEL_REDIS_KEY,
                                     ROBOT_NETWORK_INTERFACE_REDIS_KEY,
                                     ROBOT_KICK_CONSTANT_REDIS_KEY,
                                     ROBOT_ID_REDIS_KEY,
                                     ROBOT_MULTICAST_CHANNEL_REDIS_KEY,
                                     ROBOT_CHIP_PULSE_WIDTH_REDIS_KEY};

class YAMLReader
{
   public:
    explicit YAMLReader(const char* path_to_file);

    std::string getValueString(const std::string& key) const;

    int getValueInt(const std::string& key) const;

    double getValueDouble(const std::string& key) const;

   private:
    void checkContent();
    YAML::Node node_;
};
