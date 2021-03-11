#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <cstring>
#include <regex>

#include "clang-c/Index.h"
#include "yaml-cpp/yaml.h"

extern "C"
{
#include "shared/parameter/c_parameters.h"
}
/**
 * This function converts a config file names into config struct names.
 *
 * For example, a config file name maybe "example_config" gets converted
 * to "ExampleConfig"
 *
 * @throws invalid_argument If input is empty or there is a _ at the end of the input
 *
 * @pre The snake_case_input must be valid snake_case
 *
 * @param The snake_case string to convert to CamelCase
 * @return Converted string to CamelCase
 */
std::string toCamelCase(const std::string& snake_case_input)
{
    if (snake_case_input.empty() || snake_case_input.back() == '_')
    {
        throw std::invalid_argument("toCamelCase called with invalid snake_case input: " +
                                    snake_case_input);
    }

    std::string ret_str = "";
    ret_str += (char)toupper(snake_case_input[0]);

    for (unsigned i = 1; i < snake_case_input.length(); i++)
    {
        if (snake_case_input[i] == '_')
        {
            ret_str += (char)toupper(snake_case_input[i + 1]);
            i++;
        }
        else
        {
            ret_str += snake_case_input[i];
        }
    }

    return ret_str;
}


class YamlLoadFixture : public ::testing::Test
{
   protected:
    typedef struct ConfigMetadata
    {
        // the name of the config in snake_case
        std::string config_name;

        // the included configs in snake_case
        std::vector<std::string> included_configs;

        // the parameters defined by this config metadata
        std::vector<YAML::Node> parameters;

    } ConfigMetadata_t;

    /*
     * Sets up the test by loading the yaml files in the config folder.
     *
     * NOTE: These tests are highly coupled with the generate_dynamic_parameters.py script
     * and the location of the config folder.
     */
    void SetUp() override
    {
        // this is loaded from bazel data
        boost::filesystem::path path("./shared/parameter/config_definitions");

        for (auto& entry : boost::filesystem::directory_iterator(path))
        {
            std::vector<YAML::Node> config = YAML::LoadAllFromFile(entry.path().string());

            ASSERT_NE(config.size(), 0);

            ConfigMetadata_t metadata;

            // Store the config name, the name of the file
            boost::filesystem::path config_path(entry.path().string());
            metadata.config_name = config_path.stem().string();

            if (config.size() == 1)
            {
                try
                {
                    // Each yaml file is separated by a --- which indicates the end
                    // of a yaml document. A new document can start below ---.
                    // Each config can have 1 or 2 documents, which may contain just
                    // a list of included configs (1 document), just parameters (1
                    // document) or both includes and parameters (2 documents).  See a
                    // yaml config for an example
                    //
                    // We try to load the first document as a parameter definitions, and
                    // if the conversion doesn't match, we load it as an include list.
                    metadata.parameters = config[0].as<std::vector<YAML::Node>>();
                }
                catch (const YAML::BadConversion& e)
                {
                    metadata.included_configs =
                        config[0]["include"].as<std::vector<std::string>>();
                }
            }

            if (config.size() == 2)
            {
                // load both the documents because they exist
                // we don't need to worry too much about format/checking validity
                // as the schema check should have done that already.
                metadata.included_configs =
                    config[0]["include"].as<std::vector<std::string>>();
                metadata.parameters = config[1].as<std::vector<YAML::Node>>();
            }

            config_file_to_metadata_map[entry.path().string()] = metadata;
        }
    }

    // store the config metadata which contains the included configs and the parameters
    // in a mapping to the config file it was loaded from
    std::map<std::string, ConfigMetadata_t> config_file_to_metadata_map;
};

TEST_F(YamlLoadFixture, TestProperGeneration)
{
    // every file should generate a config, we iterate over all the config_metadata
    // structs for each file, and use lib-clang to iterate over config.h to make sure the
    // config was generated as expected
    std::for_each(
        config_file_to_metadata_map.begin(), config_file_to_metadata_map.end(),
        [&](std::pair<std::string, ConfigMetadata_t> config_defn) {
            std::string file_path     = config_defn.first;
            ConfigMetadata_t metadata = config_defn.second;

            // From the lib-clang docs: https://clang.llvm.org/doxygen/group__CINDEX.html
            CXIndex index = clang_createIndex(0, 0);

            // Parse the translation unit
            CXTranslationUnit unit =
                clang_parseTranslationUnit(index, "shared/parameter/c/config.h", nullptr,
                                           0, nullptr, 0, CXTranslationUnit_None);

            CXCursor cursor = clang_getTranslationUnitCursor(unit);

            // We visit over all the "stuff" in the config.h file. The cursor allows us to
            // traverse the file and get information about the structs and typedefs (among
            // other things)
            //
            // For example: in example_test_config.yaml, we may have
            //
            // include:
            //  - foo_config.yaml
            //  - bar_config.yaml
            //
            // - bool:
            //      name: example_bool_param
            //      value: true
            //      constant: true
            //      description: >
            //      Can be true or false
            //
            // This data will be available in the ConfigMetadata struct. We pass that
            // struct into a recursive visitor that goes down the entire file and
            // validates that.
            //
            // 1. There is a config struct that was generated with the proper name
            // 2. That config struct has the params defined in the yaml file
            // 3. The config struct includes the configs that are defined in the include
            //
            // NOTE: This test assumes the yaml files passed the schema checks
            clang_visitChildren(
                cursor,
                [](CXCursor c, CXCursor parent, CXClientData client_data) {
                    std::string config_name =
                        toCamelCase(((ConfigMetadata_t*)client_data)->config_name);

                    std::string generated_struct_name =
                        std::string(clang_getCString(clang_getCursorSpelling(parent)));

                    // the struct member can be an included config or another parameter
                    // we make sure that it was generated correctly by finding it in the
                    // metadata
                    std::string generated_struct_member_name =
                        std::string(clang_getCString(clang_getCursorSpelling(c)));

                    if (config_name + "_s" == generated_struct_name)
                    {
                        // We found the generated config in the config.h file. We now
                        // iterate over all the parameters and the included configs in the
                        // yaml metadata to match the struct member, if there is no match,
                        // then we generated garbage
                        bool parameter_found      = false;
                        bool include_config_found = false;

                        // We do a linear search over all the parameter yaml definitions,
                        // and check if the generated_struct_member is a valid parameter
                        for (auto& param_defn :
                             ((ConfigMetadata_t*)client_data)->parameters)
                        {
                            // This is the best way to access YAML nodes, the iterator
                            // lets us look at the definition of the parameter, without
                            // explicitly knowing the type of the parameter.
                            //
                            // bool:
                            //   name: example_param
                            //   ...
                            for (YAML::iterator it = param_defn.begin();
                                 it != param_defn.end(); ++it)
                            {
                                if (it->second["name"].as<std::string>() ==
                                    generated_struct_member_name)
                                {
                                    parameter_found = true;
                                }
                            }
                        }

                        // do a linear search over all the included configs, and check if
                        // the struct member is a defined included config
                        for (auto& included_config :
                             ((ConfigMetadata_t*)client_data)->included_configs)
                        {
                            // we are converting example_config.yaml to example_config and
                            // then to ExampleConfig, to compare against the generated
                            // struct member
                            std::string yaml_config_name = toCamelCase(
                                included_config.substr(0, included_config.find(".")));

                            if (yaml_config_name == generated_struct_member_name)
                            {
                                include_config_found = true;
                            }
                        }

                        EXPECT_TRUE(parameter_found | include_config_found)
                            << config_name << " config was not generated correctly "
                            << generated_struct_member_name << " not found in yaml";
                    }
                    return CXChildVisit_Recurse;
                },
                &metadata);

            clang_disposeTranslationUnit(unit);
            clang_disposeIndex(index);
        });
}
