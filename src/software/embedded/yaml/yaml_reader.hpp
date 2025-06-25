#pragma once

#include <string>

#include "software/constants.h"
#include "yaml-cpp/yaml.h"

// A basic config reader for yaml files, this is kind of like a dictionary
class YamlReader
{
public:
    /*
     * Create YamlReader
     *
     * @param path_to_file the path to the yaml reader
     */
    explicit YamlReader(const char* path_to_file);

    /*
     * Get the field as a string
     *
     * @param key the key we are looking for
     * @return the value of the field
     */

    template<typename T> 
    inline T getValue(const std::string& key) const {
        const YAML::Node lookup = node_[key];
        CHECK(!lookup.IsNull())
            << "The key ( " << key
            << " ) you are trying to look up does not exist in YAML file!";

        return lookup.as<T>();
    }
private:
    /*
     * If the platform is set to `LIMITED`, load a placeholder YAML
     * file to prevent the thunderloop from segfaulting. Or else, use
     * the yaml file provided in `path_to_file`
     *
     * @param path_to_file: path to the yaml file we are loading
     * @return the yaml node
     */
    YAML::Node loadNode(const char* path_to_file);

    inline static const std::string default_yaml = R"(
        robot_id : 1
        channel_id : 2
        network_interface : eno1
        kick_coeff : 1.1
        kick_constant : 1.2
        chip_pulse_width : 1.3
    )";

    
    YAML::Node node_;
};
