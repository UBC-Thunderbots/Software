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



TEST(ConfigTest, TestAutogen)
{
    // TODO this is not a proper test, just have this here to explain how the system will
    // work this is how the main configuration object will be created somewhere in hl

    // This creates a shared ptr pointing to a GlobalQualityConfig
    const std::shared_ptr<ThunderbotsConfig> DynamicParametersV2Mutable =
        std::make_shared<ThunderbotsConfig>();

    // the mutable config above will only be given to the visualizer and anything else
    // that needs to update parameters. The Immutable version below will be given
    // to all other threads
    const std::shared_ptr<const ThunderbotsConfig> DynamicParametersV2Immutable =
        std::const_pointer_cast<const ThunderbotsConfig>(DynamicParametersV2Mutable);

    // Check visitor function above, is is how the visualizer could visit
    // and set each of the values, and update them
    for (auto& v : DynamicParametersV2Mutable->getParameterList())
    {
        visit_parameters(v);
    }
}
