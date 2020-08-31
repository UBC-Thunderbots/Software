#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <regex>
#include <typeinfo>

#include "clang-c/Index.h"


extern "C"
{
#include "shared/parameter_v2/c/config.h"
#include "shared/parameter_v2/c/parameter.h"
}

#include <cstring>

#include "software/util/variant_visitor/variant_visitor.h"
#include "yaml-cpp/yaml.h"

std::ostream& operator<<(std::ostream& stream, const CXString& str)
{
    stream << clang_getCString(str);
    clang_disposeString(str);
    return stream;
}

std::string toCamelCase(const std::string& input)
{
    std::string ret_str = "";
    ret_str += (char)toupper(input[0]);

    for (unsigned i = 1; i < input.length() - 1; i++)
    {
        if (input[i] == '_')
        {
            ret_str += (char)toupper(input[i + 1]);
            i++;
        }
        else
        {
            ret_str += input[i];
        }
    }

    ret_str += input[input.length() - 1];
    return ret_str;
}


class YamlLoadFixture : public ::testing::Test
{
   protected:
    typedef struct ConfigMetadata
    {
        std::string config_name;
        std::vector<std::string> included_configs;
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
        boost::filesystem::path path("./shared/parameter_v2/config/test");

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
                    // includes, just parameters (1 document) or includes and parameters
                    // (2 documents).
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

TEST_F(YamlLoadFixture, MemoryInitializationSanityCheck)
{
    const ThunderbotsConfig_t* config = app_dynamic_parameters_create();
    ASSERT_EQ(app_dynamic_parameters_getBool(config->Example->example_bool_param), true);
    ASSERT_EQ(app_dynamic_parameters_getInt(config->Example->example_int_param), 3);
    ASSERT_EQ(std::string((app_dynamic_parameters_getString(
                  config->Example->example_string_param))),
              "Hello World");
    app_dynamic_parameters_destroy(config);
}

TEST_F(YamlLoadFixture, TestProperGeneration)
{
    // every file should generate a config
    std::for_each(
        config_file_to_metadata_map.begin(), config_file_to_metadata_map.end(),
        [&](std::pair<std::string, ConfigMetadata_t> config_defn) {
            std::string file_path     = config_defn.first;
            ConfigMetadata_t metadata = config_defn.second;

            // From the libclang docs: https://clang.llvm.org/doxygen/group__CINDEX.html
            CXIndex index = clang_createIndex(0, 0);

            // Parse the translation unit
            CXTranslationUnit unit = clang_parseTranslationUnit(
                index, "shared/parameter_v2/c/config.h", nullptr, 0, nullptr, 0,
                CXTranslationUnit_None);

            CXCursor cursor = clang_getTranslationUnitCursor(unit);

            clang_visitChildren(
                cursor,
                [](CXCursor c, CXCursor parent, CXClientData client_data) {
                    std::string config_name =
                        toCamelCase(((ConfigMetadata_t*)client_data)->config_name);

                    if (config_name + "_s" ==
                        std::string(clang_getCString(clang_getCursorSpelling(parent))))
                    {
                        std::cout << "Cursor '" << clang_getCursorSpelling(c)
                                  << "' of kind '"
                                  << clang_getCursorKindSpelling(clang_getCursorKind(c))
                                  << " with parent " << clang_getCursorSpelling(parent)
                                  << std::endl;
                    }

                    return CXChildVisit_Recurse;
                },
                &metadata);

            clang_disposeTranslationUnit(unit);
            clang_disposeIndex(index);
        });
}
