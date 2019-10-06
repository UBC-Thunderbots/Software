#include "software/util/parameter/config.hpp"

#include <gtest/gtest.h>

#include <boost/variant/static_visitor.hpp>
#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>

#include "software/util/parameter/parameter.h"

// More info on why this is needed here
// https://en.cppreference.com/w/cpp/utility/variant/visit
template <class... Ts>
struct overloaded : Ts...
{
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...)->overloaded<Ts...>;

// This is how the visualizer could iterate over all the parameters
void visit_parameters(ParameterVariant paramvar)
{
    std::visit(
        overloaded{[](std::shared_ptr<Parameter<int>> arg) {
                       std::cout << arg->name() << arg->value() << std::endl;
                   },
                   [](std::shared_ptr<Parameter<bool>> arg) {
                       std::cout << arg->name() << arg->value() << std::endl;
                   },
                   [](std::shared_ptr<Parameter<std::string>> arg) {
                       std::cout << arg->name() << arg->value() << std::endl;
                   },
                   [](std::shared_ptr<Parameter<double>> arg) {
                       std::cout << arg->name() << arg->value() << std::endl;
                   },
                   // Whoever is iterating over this config, can use a MutableConfig
                   // shared ptr and access the mutable param list to update the values
                   [](std::shared_ptr<Config> arg) {
                       std::cerr << "We hit a config! : " << arg->name() << std::endl;
                       for (auto& v : arg->getParameterList())
                       {
                           visit_parameters(v);
                       }
                   }},
        paramvar);
}


void onlyNeedParametersFromFriendlyEvalConfig(
    const std::shared_ptr<FriendlyEvalConfig> conf)
{
    std::cerr << conf->weHaveNoChill()->name() << conf->weHaveNoChill()->value()
              << std::endl;
    std::cerr << conf->weSuck()->name() << conf->weSuck()->value() << std::endl;
}

TEST(ConfigTest, TestAutogen)
{
    // TODO this is not a proper test, just have this here to explain how the system will
    // work This is how the main configuration object will be created somewhere in HL

    // TODO someone suggest a better name DynamicParameters is too reminescent of ROS
    auto DynamicParametersV2 = std::make_shared<GlobalQualityConfig>();

    // The params are accessed as follows in the new system
    std::cerr << DynamicParametersV2->getFriendlyEvalConfig()->weHaveNoChill()->value();
    // Which is equivalent to
    // Util::DynamicParameters::FriendlyEvalConfig::we_have_no_chill.value();
    // in our current system

    // But the beauty of this system is that you can divide it up into chunks and pass
    // around. The function defined above onlyNeedParametersFromConfigA, does not
    // have access to any other params other than FriendlyConfigA
    onlyNeedParametersFromFriendlyEvalConfig(
        DynamicParametersV2->getFriendlyEvalConfig());

    // Check visitor function above, is is how the visualizer could visit
    // and set each of the values
    for (auto& v : DynamicParametersV2->getMutableParameterList())
    {
        visit_parameters(v);
    }
}
