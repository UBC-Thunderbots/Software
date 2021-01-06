#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <set>
#include <boost/filesystem.hpp>

#include "shared/parameter_v2/cpp_dynamic_parameters.h"
#include "software/parameter/parameter.h"
#include "software/util/variant_visitor/variant_visitor.h"
#include "yaml-cpp/yaml.h"

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

/**
 * Finds if the given string is a valid parameter type
 * 
 * @param str the string
 * @return true if string is a valid parameter type, false otherwise
 */
bool isValidParamType(const std::string& str) {
    const static std::set<std::string> types = {"bool", "int", "uint", "float", "string", "enum", "factory"};
    return types.find(str) != types.end();
}

/**
 * Finds the parameter node (which could be an included config) within a config node
 * 
 * @param config the config to look in
 * @param param_name the name of the parameter to be looked for
 * @return the parameter node if found, a null node otherwise
 */
YAML::Node findParamNode(const YAML::Node& config, const std::string& param_name) {
    // If a config does not include other configs, it contains its paramters as a list. Otherwise,
    // it is a map where the key is an index number for non-config parameters, and the config names for included configs.
    if (config.IsSequence()) {
       // If a sequence, there are no included configs
       for (YAML::const_iterator config_it = config.begin(); config_it != config.end(); config_it++) {
        const YAML::Node param_node = *config_it;
        if (param_node.begin()->second["name"].as<std::string>() == param_name) {
            return param_node;
        }
    }
    } else if (config.IsMap()) {
        for (YAML::const_iterator config_it = config.begin(); config_it != config.end(); config_it++) {
            const std::string param_index = config_it->first.as<std::string>();
            const YAML::Node param_node = config_it->second;

            if (param_index == param_name) {
                // Must be the included config we are looking for
                return param_node;
            } else if (param_node.IsSequence()) {
                // Only a config can be a sequence type, and since it is not the included config we are looking for, continue
                continue;
            }

            YAML::const_iterator param_it = param_node.begin();
            if (isValidParamType(param_it->first.as<std::string>()) && param_it->second["name"].as<std::string>() == param_name) {
                return param_node;
            }
        }
    }

    // Could not find the node, so return a dummy node which will fail tests
    return YAML::Node();
}

class YamlLoadFixture : public ::testing::Test
{
    protected:
    /*
     * Sets up the test by loading the yaml files in the config folder.
     *
     * NOTE: These tests are highly coupled with the generate_dynamic_parameters.py script
     * and the location of the config folder.
     */
    void SetUp() override
    {
        // this is loaded from bazel data
        boost::filesystem::path path("./shared/parameter_v2/config_definitions/test");

        for (auto& entry : boost::filesystem::directory_iterator(path))
        {
            std::vector<YAML::Node> config = YAML::LoadAllFromFile(entry.path().string());

            ASSERT_NE(config.size(), 0);

            // Store the config name, the name of the file
            boost::filesystem::path config_path(entry.path().string());
            std::string config_name = toCamelCase(config_path.stem().string());
            YAML::Node current_config_yaml;

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
                    current_config_yaml = config[0]; // Convert this to function
                }
                catch (const YAML::BadConversion& e)
                {
                    config_name_to_includes_map[config_name] = config[0]["include"].as<std::vector<std::string>>();
                }
            } else if (config.size() == 2)
            {
                // load both the documents because they exist
                // we don't need to worry too much about format/checking validity
                // as the schema check should have done that already.
                config_name_to_includes_map[config_name] = config[0]["include"].as<std::vector<std::string>>();
                current_config_yaml = config[1];
            }

            config_yaml["ThunderbotsConfigNew"][config_name] = current_config_yaml;
        }

        for (auto const& p : config_name_to_includes_map)
        {
            for (auto const& included_config : p.second) {
                boost::filesystem::path included_config_path(included_config);
                std::string included_config_name = toCamelCase(included_config_path.stem().string());
                config_yaml["ThunderbotsConfigNew"][p.first][included_config_name] = config_yaml["ThunderbotsConfigNew"][included_config_name];
            }
        }
    }

    // store the config metadata which contains the included configs and the parameters
    // in a mapping to the config file it was loaded from
    YAML::Node config_yaml;
    std::map<std::string, std::vector<std::string>> config_name_to_includes_map;
};

class TestAutogenParameterList : public YamlLoadFixture
{
  public:
   /*
    * Tests that the generated parameterlist tree is identical to the yaml tree
    * Visits each config in the top level config and descends (sort of depth first)
    *
    * NOTE: at any given time this test may fail due to the key not existing in the yaml
    * dictionary which indicates an error w/ the generate script.
    */
   void visit_parameters(ParameterVariant paramvar, const YAML::Node& current_config)
   {
       std::visit(overload{[&](std::shared_ptr<const Parameter<int>> param) {
                               assert_parameter<int>(param, current_config);
                           },
                           [&](std::shared_ptr<const Parameter<bool>> param) {
                               assert_parameter<bool>(param, current_config);
                           },
                           [&](std::shared_ptr<const Parameter<std::string>> param) {
                               assert_parameter<std::string>(param, current_config);
                           },
                           [&](std::shared_ptr<const Parameter<double>> param) {
                               assert_parameter<double>(param, current_config);
                           },
                           [&](std::shared_ptr<const NumericParameter<unsigned>> param) {
                               assert_parameter<unsigned>(param, current_config);
                           },
                           [&](std::shared_ptr<const Config> param) {
                               for (auto& v : param->getParameterList())
                               {
                                   visit_parameters(v, findParamNode(current_config, param->name()));
                               }
                           }},
                  paramvar);
   }

   /*
    * Function takes a param and the corresponding yaml description
    * and checks that the yaml was generated correctly into the expected
    * parameter.
    *
    * Interally asserts and creates failures
    */
   template <typename T>
   void assert_parameter(const std::shared_ptr<const Parameter<T>>& param,
                         const YAML::Node& config_node)
   {
       try
       {
           const auto& param_description = findParamNode(config_node, param->name());
           // make sure the default value matches, accessing the yaml node with an
           // invalid key will fail the test by default
           ASSERT_EQ(param_description.begin()->second["value"].template as<T>(), param->value());
       }
       catch (...)
       {
           ADD_FAILURE() << param->name() << " didn't generate properly!";
       }
   }
};

TEST_F(TestAutogenParameterList, DynamicParametersTest)
{
   // This creates a shared ptr pointing to a ThunderbotsConfig which can be mutated
   const std::shared_ptr<ThunderbotsConfigNew> MutableDynamicParameters =
       std::make_shared<ThunderbotsConfigNew>();

   // This creates an immutable ThunderbotsConfig w/ proper const correctnesss
   const std::shared_ptr<const ThunderbotsConfigNew> DynamicParameters =
       std::const_pointer_cast<const ThunderbotsConfigNew>(MutableDynamicParameters);

   visit_parameters(DynamicParameters, config_yaml);
}

class TestParameterMutation : public YamlLoadFixture
{
  public:
   /*
    * Visits each parameter and mutates on the following policy
    *
    * std::string gets "test" appended at the end
    * int gets incremented by 4
    * double gets incremented by 2
    * bool gets flipped
    *
    * NOTE: at any given time this test may fail due to the key not existing in the yaml
    * dictionary which indicates an error w/ the generate script.
    */
   void mutate_all_parameters(MutableParameterVariant paramvar)
   {
       std::visit(overload{[&](std::shared_ptr<Parameter<int>> param) {
                               param->setValue(param->value() + 4);
                           },
                           [&](std::shared_ptr<Parameter<bool>> param) {
                               param->setValue(!param->value());
                           },
                           [&](std::shared_ptr<Parameter<std::string>> param) {
                               param->setValue(param->value() + "test");
                           },
                           [&](std::shared_ptr<Parameter<double>> param) {
                               param->setValue(param->value() - 2.0);
                           },
                           [&](std::shared_ptr<NumericParameter<unsigned>> param) {
                               param->setValue(param->value() + 3);
                           },
                           [&](std::shared_ptr<EnumeratedParameter<std::string>> param) {
                               if (param->name() == "example_factory_param") {
                                   param->setValue("ExamplePlay");
                               } else {
                                   param->setValue("STOP");
                               }
                           },
                           [&](std::shared_ptr<Config> param) {
                               for (auto& v : param->getMutableParameterList())
                               {
                                   mutate_all_parameters(v);
                               }
                           }},
                  paramvar);
   }

   /*
    * Tests that the mutations in the mutable parameter list tree are propagated to
    * the immutable tree
    *
    * NOTE: at any given time this test may fail due to the key not existing in the yaml
    * dictionary which indicates an error w/ the generate script.
    */
   void assert_mutation(ParameterVariant paramvar, const YAML::Node& current_config)
   {
       std::visit(
           overload{
               [&](std::shared_ptr<const Parameter<int>> param) {
                   ASSERT_EQ(current_config[param->name()]["default"].as<int>(),
                             param->value() - 4);
               },
               [&](std::shared_ptr<const Parameter<bool>> param) {
                   ASSERT_EQ(current_config[param->name()]["default"].as<bool>(),
                             !param->value());
               },
               [&](std::shared_ptr<const Parameter<std::string>> param) {
                   ASSERT_EQ(current_config[param->name()]["default"].as<std::string>() +
                                 "test",
                             param->value());
               },
               [&](std::shared_ptr<const Parameter<double>> param) {
                   ASSERT_NEAR(current_config[param->name()]["default"].as<double>(),
                               param->value() + 2.0, 1E-10);
               },
               [&](std::shared_ptr<const NumericParameter<unsigned>> param) {
                   // This will be tested as part of the new parameter system (issue
                   // #1298)
               },
               [&](std::shared_ptr<const Config> param) {
                   for (auto& v : param->getParameterList())
                   {
                       assert_mutation(v, current_config[param->name()]);
                   }
               }},
           paramvar);
   }
};

TEST_F(TestParameterMutation, DynamicParametersTest)
{
    // This creates a shared ptr pointing to a ThunderbotsConfig which can be mutated
    const std::shared_ptr<ThunderbotsConfigNew> MutableDynamicParameters =
        std::make_shared<ThunderbotsConfigNew>();

   // This creates an immutable ThunderbotsConfig w/ proper const correctnesss
    const std::shared_ptr<const ThunderbotsConfigNew> DynamicParameters =
        std::const_pointer_cast<const ThunderbotsConfigNew>(MutableDynamicParameters);

    mutate_all_parameters(MutableDynamicParameters);
    //assert_mutation(DynamicParameters, config_yaml);
}

