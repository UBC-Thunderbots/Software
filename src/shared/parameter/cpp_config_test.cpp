#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <set>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "shared/parameter/parameter.h"
#include "software/ai/hl/stp/play/halt_play.h"
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
bool isValidParamType(const std::string& str)
{
    const static std::set<std::string> types = {"bool",   "int",  "double",
                                                "string", "enum", "factory"};
    return types.find(str) != types.end();
}

/**
 * Gets the value of a field from a parameter node
 *
 * @param param_node the YAML node of parameter
 * @param field the name of the field
 */
template <typename T>
T getParamField(const YAML::Node& param_node, const std::string& field)
{
    return param_node.begin()->second[field].template as<T>();
}

/**
 * Checks if a given parameter node is constant
 *
 * @param param_node the YAML node of parameter
 * @return true if the node contains constant field and the field is true, false otherwise
 */
bool isParamConstant(const YAML::Node& param_node)
{
    return param_node.begin()->second["constant"] &&
           getParamField<bool>(param_node, "constant");
}

/**
 * Finds the parameter node (which could be an included config) within a config node
 *
 * @param config the config to look in
 * @param param_name the name of the parameter to be looked for
 * @return the parameter node if found, a null node otherwise
 */
YAML::Node findParamNode(const YAML::Node& config, const std::string& param_name)
{
    // If a config does not include other configs, it contains its parameters as a list.
    // Otherwise, it is a map where the key is an index number for non-config parameters,
    // and the config names for included configs.
    if (config.IsSequence())
    {
        // If a sequence, there are no included configs
        for (YAML::const_iterator config_it = config.begin(); config_it != config.end();
             config_it++)
        {
            const YAML::Node param_node = *config_it;
            if (getParamField<std::string>(param_node, "name") == param_name)
            {
                return param_node;
            }
        }
    }
    else if (config.IsMap())
    {
        for (YAML::const_iterator config_it = config.begin(); config_it != config.end();
             config_it++)
        {
            const std::string param_index = config_it->first.as<std::string>();
            const YAML::Node param_node   = config_it->second;

            if (param_index == param_name)
            {
                // Must be the included config we are looking for
                return param_node;
            }
            else if (param_node.IsSequence())
            {
                // Only a config can be a sequence type, and since it is not the included
                // config we are looking for, continue
                continue;
            }

            if (isValidParamType(param_node.begin()->first.as<std::string>()) &&
                getParamField<std::string>(param_node, "name") == param_name)
            {
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
        boost::filesystem::path path("./shared/parameter/config_definitions/");

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
                    // We try to load the first document as a include list, and
                    // if the conversion doesn't match, we load it as parameters.
                    config_name_to_includes_map[config_name] =
                        config[0]["include"].as<std::vector<std::string>>();
                }
                catch (const YAML::BadConversion& e)
                {
                    current_config_yaml = config[0];
                }
            }
            else if (config.size() == 2)
            {
                // load both the documents because they exist
                // we don't need to worry too much about format/checking validity
                // as the schema check should have done that already.
                config_name_to_includes_map[config_name] =
                    config[0]["include"].as<std::vector<std::string>>();
                current_config_yaml = config[1];
            }

            config_yaml["ThunderbotsConfig"][config_name] = current_config_yaml;
        }

        for (auto const& p : config_name_to_includes_map)
        {
            for (auto const& included_config : p.second)
            {
                boost::filesystem::path included_config_path(included_config);
                std::string included_config_name =
                    toCamelCase(included_config_path.stem().string());
                config_yaml["ThunderbotsConfig"][p.first][included_config_name] =
                    config_yaml["ThunderbotsConfig"][included_config_name];
            }
        }
    }

    YAML::Node config_yaml;
    std::map<std::string, std::vector<std::string>> config_name_to_includes_map;
};

class TestAutogenParameterList : public YamlLoadFixture
{
   public:
    /*
     * Tests that the generated parameterlist tree is identical to the yaml tree
     * Visits each config in the top level config and descends (sort of depth first)
     */
    void visit_parameters(ParameterVariant paramvar, const YAML::Node& current_config)
    {
        std::visit(overload{[&](std::shared_ptr<const Parameter<int>> param) {
                                assertParameter<int>(param, current_config);
                            },
                            [&](std::shared_ptr<const Parameter<bool>> param) {
                                assertParameter<bool>(param, current_config);
                            },
                            [&](std::shared_ptr<const Parameter<std::string>> param) {
                                assertParameter<std::string>(param, current_config);
                            },
                            [&](std::shared_ptr<const Parameter<double>> param) {
                                assertParameter<double>(param, current_config);
                            },
                            [&](std::shared_ptr<const NumericParameter<unsigned>> param) {
                                assertParameter<unsigned>(param, current_config);
                            },
                            [&](std::shared_ptr<const Config> param) {
                                const YAML::Node param_node =
                                    findParamNode(current_config, param->name());
                                for (auto& v : param->getParameterList())
                                {
                                    visit_parameters(v, param_node);
                                }
                            }},
                   paramvar);
    }

    /*
     * Function takes a param and the config node that contains it, and
     * checks that the yaml was generated correctly into the expected
     * parameter.
     *
     * Internally asserts and creates failures
     */
    template <typename T>
    void assertParameter(const std::shared_ptr<const Parameter<T>>& param,
                         const YAML::Node& config_node)
    {
        try
        {
            const YAML::Node param_node = findParamNode(config_node, param->name());
            // make sure the default value matches, accessing the yaml node with an
            // invalid key will fail the test by default
            ASSERT_EQ(getParamField<T>(param_node, "value"), param->value());
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
    const std::shared_ptr<ThunderbotsConfig> MutableDynamicParameters =
        std::make_shared<ThunderbotsConfig>();

    // This creates an immutable ThunderbotsConfig w/ proper const correctnesss
    const std::shared_ptr<const ThunderbotsConfig> DynamicParameters =
        std::const_pointer_cast<const ThunderbotsConfig>(MutableDynamicParameters);

    visit_parameters(DynamicParameters, config_yaml);
}

class TestParameterMutation : public YamlLoadFixture
{
   public:
    /*
     * Visits each parameter and mutates on the following policy
     *
     * std::string gets "test" appended at the end
     * int gets set to max
     * double gets set to max
     * bool gets flipped
     *
     * Some parameters such as example_enum_param has a specific value set
     */
    void mutateAllParameters(MutableParameterVariant paramvar)
    {
        static std::set<MutableParameterVariant> visited;
        visited.insert(paramvar);
        std::visit(overload{[&](std::shared_ptr<NumericParameter<int>> param) {
                                param->setValue(param->getMax());
                            },
                            [&](std::shared_ptr<Parameter<bool>> param) {
                                param->setValue(!param->value());
                            },
                            [&](std::shared_ptr<Parameter<std::string>> param) {
                                param->setValue(param->value() + "test");
                            },
                            [&](std::shared_ptr<NumericParameter<double>> param) {
                                param->setValue(param->getMax());
                            },
                            [&](std::shared_ptr<NumericParameter<unsigned>> param) {
                                param->setValue(param->getMax());
                            },
                            [&](std::shared_ptr<EnumeratedParameter<std::string>> param) {
                                if (param->name() == "example_factory_param")
                                {
                                    param->setValue("ExamplePlay");
                                }
                                else
                                {
                                    param->setValue("STOP");
                                }
                            },
                            [&](std::shared_ptr<Config> param) {
                                for (auto& v : param->getMutableParameterList())
                                {
                                    if (visited.find(v) == visited.end())
                                    {
                                        // Only mutate once per parameter
                                        mutateAllParameters(v);
                                    }
                                }
                            }},
                   paramvar);
    }

    /*
     * Tests that the mutations in the mutable parameter list tree are propagated to
     * the immutable tree, and that constant parameters have not changed
     */
    void assertMutation(ParameterVariant paramvar, const YAML::Node& current_config)
    {
        std::visit(
            overload{[&](std::shared_ptr<const NumericParameter<int>> param) {
                         const YAML::Node param_node =
                             findParamNode(current_config, param->name());
                         if (!(isParamConstant(param_node)))
                         {
                             ASSERT_EQ(getParamField<int>(param_node, "max"),
                                       param->value());
                         }
                         else
                         {
                             ASSERT_EQ(getParamField<int>(param_node, "value"),
                                       param->value());
                         }
                     },
                     [&](std::shared_ptr<const Parameter<bool>> param) {
                         const YAML::Node param_node =
                             findParamNode(current_config, param->name());
                         if (!(isParamConstant(param_node)))
                         {
                             ASSERT_EQ(!getParamField<bool>(param_node, "value"),
                                       param->value());
                         }
                         else
                         {
                             ASSERT_EQ(getParamField<bool>(param_node, "value"),
                                       param->value());
                         }
                     },
                     [&](std::shared_ptr<const Parameter<std::string>> param) {
                         const YAML::Node param_node =
                             findParamNode(current_config, param->name());
                         if (!(isParamConstant(param_node)))
                         {
                             ASSERT_EQ(
                                 getParamField<std::string>(param_node, "value") + "test",
                                 param->value());
                         }
                         else
                         {
                             ASSERT_EQ(getParamField<std::string>(param_node, "value"),
                                       param->value());
                         }
                     },
                     [&](std::shared_ptr<const NumericParameter<double>> param) {
                         const YAML::Node param_node =
                             findParamNode(current_config, param->name());
                         if (!(isParamConstant(param_node)))
                         {
                             ASSERT_NEAR(getParamField<double>(param_node, "max"),
                                         param->value(), 1E-10);
                         }
                         else
                         {
                             ASSERT_EQ(getParamField<double>(param_node, "value"),
                                       param->value());
                         }
                     },
                     [&](std::shared_ptr<const NumericParameter<unsigned>> param) {
                         const YAML::Node param_node =
                             findParamNode(current_config, param->name());
                         if (!(isParamConstant(param_node)))
                         {
                             ASSERT_EQ(getParamField<unsigned>(param_node, "max"),
                                       param->value());
                         }
                         else
                         {
                             ASSERT_EQ(getParamField<unsigned>(param_node, "value"),
                                       param->value());
                         }
                     },
                     [&](std::shared_ptr<const EnumeratedParameter<std::string>> param) {
                         const YAML::Node param_node =
                             findParamNode(current_config, param->name());
                         if (!(isParamConstant(param_node)))
                         {
                             if (param->name() == "example_enum_param")
                             {
                                 ASSERT_EQ("STOP", param->value());
                             }
                             else if (param->name() == "example_factory_param")
                             {
                                 ASSERT_EQ("ExamplePlay", param->value());
                             }
                         }
                         else
                         {
                             ASSERT_EQ(getParamField<std::string>(param_node, "value"),
                                       param->value());
                         }
                     },
                     [&](std::shared_ptr<const Config> param) {
                         const YAML::Node param_node =
                             findParamNode(current_config, param->name());
                         for (auto& v : param->getParameterList())
                         {
                             assertMutation(v, param_node);
                         }
                     }},
            paramvar);
    }
};

TEST_F(TestParameterMutation, DynamicParametersTest)
{
    // This creates a shared ptr pointing to a ThunderbotsConfig which can be mutated
    const std::shared_ptr<ThunderbotsConfig> mutable_dynamic_parameters =
        std::make_shared<ThunderbotsConfig>();

    // This creates an immutable ThunderbotsConfig w/ proper const correctnesss
    const std::shared_ptr<const ThunderbotsConfig> dynamic_params =
        std::const_pointer_cast<const ThunderbotsConfig>(mutable_dynamic_parameters);

    mutateAllParameters(mutable_dynamic_parameters);
    assertMutation(dynamic_params, config_yaml);
}

TEST_F(TestParameterMutation, ProtoGenerationTest)
{
    // This creates a shared ptr pointing to a ThunderbotsConfig which can be mutated
    const std::shared_ptr<ThunderbotsConfig> mutable_dynamic_parameters =
        std::make_shared<ThunderbotsConfig>();

    // This creates an immutable ThunderbotsConfig w/ proper const correctnesss
    const std::shared_ptr<const ThunderbotsConfig> dynamic_params =
        std::const_pointer_cast<const ThunderbotsConfig>(mutable_dynamic_parameters);

    // Mutate all the parameters
    mutateAllParameters(mutable_dynamic_parameters);

    // Save as a proto
    auto proto = mutable_dynamic_parameters->toProto();

    // Create a new top level config
    const std::shared_ptr<ThunderbotsConfig> mutable_dynamic_param_loaded_from_proto =
        std::make_shared<ThunderbotsConfig>();

    // Load top level config from the proto and make sure that the mutations across all
    // configs are reflected
    mutable_dynamic_param_loaded_from_proto->loadFromProto(proto);
    assertMutation(mutable_dynamic_param_loaded_from_proto, config_yaml);
}
