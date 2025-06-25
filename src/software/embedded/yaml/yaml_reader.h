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
    T getValue(const std::string& key) const;

   private:
    YAML::Node node_;
};

extern template int YamlReader::getValue<int>(const std::string& key) const;  
extern template std::string YamlReader::getValue<std::string>(const std::string& key) const;  
extern template double YamlReader::getValue<double>(const std::string& key) const;  
