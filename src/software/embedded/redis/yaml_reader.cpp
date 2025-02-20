#include "yaml_reader.h"

#include <g3log/g3log.hpp>

YAMLReader::YAMLReader(const char* path_to_file) : node_(YAML::LoadFile(path_to_file))
{
    checkContent();
}

std::string YAMLReader::getValueString(const std::string& key) const
{
    return node_[key].as<std::string>();
}

int YAMLReader::getValueInt(const std::string& key) const
{
    return node_[key].as<int>();
}

double YAMLReader::getValueDouble(const std::string& key) const
{
    return node_[key].as<double>();
}

void YAMLReader::checkContent()
{
    CHECK(!node_.IsNull()) << "failed to read the node";
    for (const std::string key : KEYS_TO_CHECK)
    {
        CHECK(!node_[key].IsNull()) << "You are missing a requried key: " << key;
    }
}
