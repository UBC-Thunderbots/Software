#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include "software/parameter/dynamic_parameters.h"
#include "software/parameter/parameter.h"
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
        boost::filesystem::path path("./software/parameter/config");

        for (auto& entry : boost::filesystem::directory_iterator(path))
        {
            YAML::Node config = YAML::LoadFile(entry.path().string());
            for (auto it = config.begin(); it != config.end(); it++)
            {
                // this creates an internal representation of the yaml files similar
                // to how generate_parameters.py creates its internal dictionary
                // before generating the required code.
                config_yaml["ThunderbotsConfig"][it->first.as<std::string>()] =
                    it->second;
            }
        }
    }

    // stores the yaml that should have the same structure
    // as the config generated
    YAML::Node config_yaml;
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
                                // This will be tested as part of the new parameter system
                                // (issue #1298)
                            },
                            [&](std::shared_ptr<const Config> param) {
                                for (auto& v : param->getParameterList())
                                {
                                    visit_parameters(v, current_config[param->name()]);
                                }
                            }},
                   paramvar);
    }

    /*
     * Function takes a param and the corresponding yaml description
     * and checks that the yaml was generated correctly into the expected
     * parameter.
     *
     * Internally asserts and creates failures
     */
    template <typename T>
    void assert_parameter(const std::shared_ptr<const Parameter<T>>& param,
                          const YAML::Node& param_description)
    {
        try
        {
            // make sure the default value matches, accessing the yaml node with an
            // invalid key will fail the test by default
            ASSERT_EQ(param_description[param->name()]["default"].template as<T>(),
                      param->value());
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
                                // This will be tested as part of the new parameter system
                                // (issue #1298)
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
    const std::shared_ptr<ThunderbotsConfig> MutableDynamicParameters =
        std::make_shared<ThunderbotsConfig>();

    // This creates an immutable ThunderbotsConfig w/ proper const correctnesss
    const std::shared_ptr<const ThunderbotsConfig> DynamicParameters =
        std::const_pointer_cast<const ThunderbotsConfig>(MutableDynamicParameters);

    mutate_all_parameters(MutableDynamicParameters);
    assert_mutation(DynamicParameters, config_yaml);
}
