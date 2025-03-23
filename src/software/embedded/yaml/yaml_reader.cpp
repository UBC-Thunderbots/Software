#include "yaml_reader.h"

#include <g3log/g3log.hpp>

YamlReader::YamlReader(const char* path_to_file) : node_(YAML::LoadFile(path_to_file)) {}

std::string YamlReader::getValueString(const std::string& key) const
{
    const YAML::Node lookup = node_[key];
    CHECK(!lookup.IsNull())
        << "The key ( " << key
        << " ) you are trying to look up does not exist in YAML file!";

    return lookup.as<std::string>();
}

int YamlReader::getValueInt(const std::string& key) const
{
    const YAML::Node lookup = node_[key];
    CHECK(!lookup.IsNull())
        << "The key ( " << key
        << " ) you are trying to look up does not exist in YAML file!";

    return lookup.as<int>();
}

double YamlReader::getValueDouble(const std::string& key) const
{
    // To not give cryptic exception messages, because if the key doesn't exist
    // an exception would just be thrown regardless!

    const YAML::Node lookup = node_[key];
    CHECK(!lookup.IsNull())
        << "The key ( " << key
        << " ) you are trying to look up does not exist in YAML file!";

    return lookup.as<double>();
}
