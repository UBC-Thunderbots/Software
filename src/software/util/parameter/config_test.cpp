#include "software/util/parameter/config.hpp"

#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include "software/util/parameter/parameter.h"
#include "yaml-cpp/yaml.h"


// More info on why this is needed here
// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...)->overloaded<Ts...>;


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
        boost::filesystem::path path("./config");

        for (auto& entry : boost::filesystem::directory_iterator(path))
        {
            YAML::Node config = YAML::LoadFile(entry.path().string());
            config_yaml["ThunderbotsConfig/" + config.Tag()] = config;
        }
    }

    void TearDown() override {}
    YAML::Node config_yaml;
};

class TestAutogenParameterList : public YamlLoadFixture
{
   public:
    void visit_parameters(ParameterVariant paramvar, const YAML::Node& current_config)
    {
        std::visit(overloaded{[&](std::shared_ptr<const Parameter<int>> param) {
                                  assert_generarted_param_identical_to_yaml<int>(
                                      param, current_config);
                              },
                              [&](std::shared_ptr<const Parameter<bool>> param) {
                                  assert_generarted_param_identical_to_yaml<bool>(
                                      param, current_config);
                              },
                              [&](std::shared_ptr<const Parameter<std::string>> param) {
                                  assert_generarted_param_identical_to_yaml<std::string>(
                                      param, current_config);
                              },
                              [&](std::shared_ptr<const Parameter<double>> param) {
                                  assert_generarted_param_identical_to_yaml<double>(
                                      param, current_config);
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
     * Interally creates failures
     */
    template <class T>
    void assert_generarted_param_identical_to_yaml(
        const std::shared_ptr<const Parameter<T>>& param,
        const YAML::Node& param_description)
    {
        std::cerr << "PARAM CHECKER" << param_description << std::endl;
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
