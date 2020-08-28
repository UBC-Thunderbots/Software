#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>


extern "C"
{
#include "shared/parameter_v2/c/config.h"
#include "shared/parameter_v2/c/parameter.h"
}

#include "software/util/variant_visitor/variant_visitor.h"
#include "yaml-cpp/yaml.h"

class YamlLoadFixture : public ::testing::Test
{
    /*
     * This fixture is responsible for loading all the yaml files in the
     * config directory and storing them into an accessible config_yaml vector
     * which will be used to compare the generated config and the specified config
     *
     */
   protected:
    /*
     * Sets up the test by loading the yaml files in the config folder.
     *
     * NOTE: These tests are highly coupled with the generate_parameters.py script
     * and the location of the config folder.
     */
    void SetUp() override
    {
        // this is loaded from bazel data
        boost::filesystem::path path("./shared/parameter_v2/config/examples");

        for (auto& entry : boost::filesystem::directory_iterator(path))
        {
            std::cerr << entry.path().string() << std::endl;
            YAML::Node config = YAML::LoadFile(entry.path().string());
            for (auto it = config.begin(); it != config.end(); it++)
            {
                std::cerr << "YOOT" << std::endl;
                auto test = it->as<std::vector<std::string>>();
                for (auto b : test)
                    std::cerr << b << std::endl;
                std::cerr << "YOOD" << std::endl;
                // YAML::Node key = it->first;
                // YAML::Node value = it->second;
                // std::cerr<<key<<std::endl;
                // std::cerr<<value<<std::endl;
            }
        }
    }

    // stores the yaml that should have the same structure
    // as the config generated
    YAML::Node config_yaml;
};

TEST_F(YamlLoadFixture, MemoryInitializationSanityCheck)
{
    const ThunderbotsConfig_t* config = app_dynamic_parameters_create();
    app_dynamic_parameters_destroy(config);
}
