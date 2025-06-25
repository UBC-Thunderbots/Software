#include "software/logger/logger.h"
#include "software/embedded/platform.h"
#include "software/embedded/constants/constants.h"
#include "yaml_reader.hpp"


YamlReader::YamlReader(const char* path_to_file)
    : node_(loadNode(path_to_file)) { }

YAML::Node YamlReader::loadNode(const char* path_to_file){
    if constexpr (PLATFORM == Platform::LIMITED_BUILD)
        return YAML::Load(default_yaml); 
    

    return YAML::LoadFile(path_to_file); 
}
