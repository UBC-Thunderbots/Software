#pragma once

#include <functional>
#include <include/boost/sml.hpp>
#include <queue>

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
