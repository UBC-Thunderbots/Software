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
    std::string getValueString(const std::string& key) const;

    /*
     * Get the field as an int
     *
     * @param key the key we are looking for
     * @return the value of the field
     */
    int getValueInt(const std::string& key) const;

    /*
     * Get the field as a double
     *
     * @param key the key we are looking for
     * @return the value of the field
     */
    double getValueDouble(const std::string& key) const;

   private:
    YAML::Node node_;
};
