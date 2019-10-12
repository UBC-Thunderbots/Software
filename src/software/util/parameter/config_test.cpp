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
    const std::shared_ptr<const FriendlyEvalConfig> conf)
{
    std::cerr << conf->weHaveNoChill()->name() << conf->weHaveNoChill()->value()
              << std::endl;
    std::cerr << conf->weSuck()->name() << conf->weSuck()->value() << std::endl;

    // this wont compile
    // conf->weSuck()->setValue(false);
}

TEST(ConfigTest, TestAutogen)
{
    // TODO this is not a proper test, just have this here to explain how the system will
    // work this is how the main configuration object will be created somewhere in hl

    // This creates a shared ptr pointing to a GlobalQualityConfig
    const std::shared_ptr<GlobalQualityConfig> DynamicParametersV2Mutable =
        std::make_shared<GlobalQualityConfig>();

    // accessing parameter values works as expected on the MutableConfig
    std::cerr << DynamicParametersV2Mutable->getFriendlyEvalConfigMutable()
                     ->weHaveNoChillMutable()
                     ->value();

    // Which is Util::DynamicParameters::FriendlyEvalConfig::we_have_no_chill.value();
    // in our current system, and we can also update the values with this
    DynamicParametersV2Mutable->getFriendlyEvalConfigMutable()
        ->weHaveNoChillMutable()
        ->setValue(true);

    // the mutable config above will only be given to the visualizer and anything else
    // that needs to update parameters. The Immutable version below will be given
    // to all other threads
    const std::shared_ptr<const GlobalQualityConfig> DynamicParametersV2Immutable =
        std::const_pointer_cast<const GlobalQualityConfig>(DynamicParametersV2Mutable);

    // const_pointer_cast does exactly what we want it to do, it takes a const pointer to
    // a GlobalQualityConfig and changes it to a const pointer to a const
    // GlobalQualityConfig AND still shares the same internal representiation

    // Example usage of Mutable vs Immutable: The changes through the mutable version be
    // seen in the the Immutable version (we don't want a copy)
    DynamicParametersV2Mutable->getFriendlyEvalConfigMutable()->weSuckMutable()->setValue(
        true);
    ASSERT_EQ(DynamicParametersV2Immutable->getFriendlyEvalConfig()->weSuck()->value(),
              true);

    DynamicParametersV2Mutable->getFriendlyEvalConfigMutable()->weSuckMutable()->setValue(false);
    ASSERT_EQ(DynamicParametersV2Immutable->getFriendlyEvalConfig()->weSuck()->value(), false);

    // NOTE: the following commands will not compile for example
    // DynamicParametersV2Immutable->getFriendlyEvalConfig()->weHaveNoChill()->setValue(true);
    // DynamicParametersV2Immutable->getFriendlyEvalConfigMutable()->weSuckMutable()->setValue(false);
    // DynamicParametersV2Immutable->getFriendlyEvalConfig()->weSuckMutable()->setValue(false);

    // But the beauty of this system is that you can divide it up into chunks and pass
    // around. The function defined above onlyNeedParametersFromConfigA, does not
    // have access to any other params other than FriendlyConfigA
    onlyNeedParametersFromFriendlyEvalConfig(DynamicParametersV2Immutable->getFriendlyEvalConfig());


    // Check visitor function above, is is how the visualizer could visit
    // and set each of the values, and update them
    for (auto& v : DynamicParametersV2Mutable->getMutableParameterList())
    {
        visit_parameters(v);
    }
}
