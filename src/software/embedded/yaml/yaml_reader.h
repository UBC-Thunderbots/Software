#pragma once

#include <string>

#include "software/constants.h"
#include "yaml-cpp/yaml.h"

class YamlReader
{
   public:
    explicit YamlReader(const char* path_to_file);

    std::string getValueString(const std::string& key) const;

    int getValueInt(const std::string& key) const;

    double getValueDouble(const std::string& key) const;

   private:
    YAML::Node node_;
};
