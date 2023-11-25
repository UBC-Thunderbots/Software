#pragma once

#include <functional>
#include <include/boost/sml.hpp>
#include <queue>

#include "software/util/typename/typename.h"

/**
 * The Tactic FSM framework uses the [SML library](https://github.com/boost-ext/sml), and
 * aims to create a readable style of FSM to implement tactic gameplay. See the MoveTactic
 * for an example of how to implement a tactic using this framework
 */

// An alias for an FSM
template <class T>
using FSM = boost::sml::sm<T, boost::sml::process_queue<std::queue>>;

/**
 * Defines an SML state wrapper around a class/struct
 *
 * @param STATE The state class/struct
 */
#define DEFINE_SML_STATE(STATE) const auto STATE##_S = boost::sml::state<STATE>;

/**
 * Defines an SML event wrapper around a class/struct
 *
 * @param EVENT The event class/struct
 */
#define DEFINE_SML_EVENT(EVENT) const auto EVENT##_E = boost::sml::event<EVENT>;

/**
 * Defines lambda wrapper around a function that can be used as an SML guard
 *
 * @param FUNCTION The function to turn into a lambda
 */
#define DEFINE_SML_GUARD(FUNCTION)                                                       \
    const auto FUNCTION##_G = [this](auto event) { return FUNCTION(event); };

/**
 * Defines lambda wrapper around a function that can be used as an SML action
 *
 * @param FUNCTION The function to turn into a lambda
 */
#define DEFINE_SML_ACTION(FUNCTION)                                                      \
    const auto FUNCTION##_A = [this](auto event) { FUNCTION(event); };

/**
 * Defines lambda wrapper around a function that can be used as an SML action for updating
 * a sub fsm
 *
 * @param FUNCTION The function to turn into a lambda
 * @param SUB_FSM The sub fsm to update
 */
#define DEFINE_SML_SUB_FSM_UPDATE_ACTION(FUNCTION, SUB_FSM)                              \
    const auto FUNCTION##_A = [this](auto event,                                         \
                                     back::process<SUB_FSM::Update> processEvent) {      \
        FUNCTION(event, processEvent);                                                   \
    };

/**
 * Strips extraneous information such as boost::sml template information to return
 * human-friendly text about the state
 *
 * @param s the string with the extraneous information
 *
 * @return the string without the extraneous information
 */
std::string stripFSMState(std::string s);

/**
 * Gets the current state name of the FSM
 * Adapted from https://github.com/boost-ext/sml/issues/326#issuecomment-605529165
 *
 * @param state_machine The boost sml state machine
 *
 * @return the current state name
 */
template <typename SM>
std::string getCurrentStateName(const SM& state_machine)
{
    std::string name;
    state_machine.visit_current_states([&name](const auto& state) {
        name = stripFSMState(TYPENAME(
            boost::sml::back::policies::get_state_name_t<std::decay_t<decltype(state)>>));
    });
    return name;
}

template <typename>
struct is_sub_state_machine : std::false_type
{
};

template <class T, class... Ts>
struct is_sub_state_machine<boost::sml::back::sm<boost::sml::back::sm_policy<T, Ts...>>>
    : std::true_type
{
};

template <typename>
struct state_machine_impl : std::false_type
{
};

template <class T, class... Ts>
struct state_machine_impl<boost::sml::back::sm<boost::sml::back::sm_policy<T, Ts...>>>
{
    using type = T;
};

/**
 * Gets the current sub state name of the FSM
 * Adapted from https://github.com/boost-ext/sml/issues/326#issuecomment-605529165
 *
 * @param state_machine The boost sml state machine
 *
 * @return the current sub state name
 */
template <typename SSM, typename SM>
std::string getCurrentSubStateName(const SM& state_machine)
{
    std::string name;
    state_machine.template visit_current_states<SSM>([&name,
                                                      &state_machine](const auto& state) {
        name               = stripFSMState(TYPENAME(
            boost::sml::back::policies::get_state_name_t<std::decay_t<decltype(state)>>));
        using state_repr_t = std::decay_t<decltype(state)>;
        using state_t      = typename state_repr_t::type;
        if constexpr (is_sub_state_machine<state_t>::value)
        {
            using state_machine_t = typename state_machine_impl<state_t>::type;
            name += ".";
            name += getCurrentSubStateName<decltype(boost::sml::state<state_machine_t>)>(
                state_machine);
        }
    });
    return name;
}

/**
 * Gets the current full state name of the FSM, including sub states
 * Adapted from https://github.com/boost-ext/sml/issues/326#issuecomment-605529165
 *
 * @param state_machine The boost sml state machine
 *
 * @return the current full state name
 */
template <typename SM>
std::string getCurrentFullStateName(const SM& state_machine)
{
    std::string name;
    state_machine.visit_current_states([&name, &state_machine](const auto& state) {
        name += getCurrentStateName(state_machine);
        using state_repr_t = std::decay_t<decltype(state)>;
        using state_t      = typename state_repr_t::type;
        if constexpr (is_sub_state_machine<state_t>::value)
        {
            using state_machine_t = typename state_machine_impl<state_t>::type;
            name += ".";
            name += getCurrentSubStateName<decltype(boost::sml::state<state_machine_t>)>(
                state_machine);
        }
    });
    return name;
}
