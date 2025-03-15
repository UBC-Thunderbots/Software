#include "yaml_reader.h"

#include <g3log/g3log.hpp>

YAMLReader::YAMLReader(const char* path_to_file) : node_(YAML::LoadFile(path_to_file))
{
    checkContent();
}

std::string YAMLReader::getValueString(const std::string& key) const
{
    const YAML::Node lookup = node_[key];
    CHECK(!lookup.IsNull()) << "The key ( " << key << " ) you are trying to look up does not exist in YAML file!"; 

    return lookup.as<std::string>();
}

int YAMLReader::getValueInt(const std::string& key) const
{
    const YAML::Node lookup = node_[key];
    CHECK(!lookup.IsNull()) << "The key ( " << key << " ) you are trying to look up does not exist in YAML file!"; 

    return lookup.as<int>();
}

double YAMLReader::getValueDouble(const std::string& key) const
{
    // To not give cryptic exception messages, because if the key doesn't exist
    // an exception would just be thrown regardless!

    const YAML::Node lookup = node_[key];
    CHECK(!lookup.IsNull()) << "The key ( " << key << " ) you are trying to look up does not exist in YAML file!"; 

    return lookup.as<double>();
}

void YAMLReader::checkContent()
{
    CHECK(!node_.IsNull()) << "failed to read the node";
    for (const std::string key : KEYS_TO_CHECK)
    {
        CHECK(!node_[key].IsNull()) << "You are missing a requried key: " << key;
    }
}
