#include "software/logger/logger.h"
#include "yaml_reader.h"

YamlReader::YamlReader(const char* path_to_file) : node_(YAML::LoadFile(path_to_file)) {}

template<typename T> 
T YamlReader::getValue(const std::string& key) const {
    CHECK(!node_.IsNull())
        << "The key ( " << key
        << " ) you are trying to look up does not exist in YAML file!";

    return node_.as<T>();
}
