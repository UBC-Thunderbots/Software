#pragma once

#include <functional>
#include <include/boost/sml.hpp>
#include <queue>
#include <string>
#include <map>
#include "proto/play_info_msg.pb.h"
#include "software/logger/logger.h"

#include "software/util/typename/typename.h"

/**
 * This class declares a logger to log the FSM State Changes
 * https://boost-ext.github.io/sml/examples.html#logging
 *
 * The logger follows the singleton pattern: https://stackoverflow.com/questions/13047526/difference-between-singleton-implemention-using-pointer-and-using-static-object
 * It also applies some filtering to prevent overwhelming protobuffer messages.
 */

class FSMLogger {
public:

    /**
     * Get the single logger instance
     *
     * @return the logger instance
     */
    static FSMLogger& getInstance()
    {
        static FSMLogger logger;
        return logger;
    }

    /**
     * This function is called whenever an event is processed by the state machine
     *
     * @tparam SM the state machine
     * @tparam TEvent the event
     */
    template <class SM, class TEvent>
    void log_process_event(const TEvent&)
    {
        //LOG(INFO) << "[%s][process_event] %s\n" << boost::sml::aux::get_type_name<SM>() << boost::sml::aux::get_type_name<TEvent>();
    }

    /**
     * This function is called when a guard is processed by the state machine
     *
     * @tparam SM the state machine
     * @tparam TGuard the guard
     * @tparam TEvent the event
     * @param result true if the guard passed, false if the guard failed
     */
    template <class SM, class TGuard, class TEvent>
    void log_guard(const TGuard&, const TEvent&, bool result)
    {
//        LOG(INFO) << boost::sml::aux::get_type_name<SM>() << "  " << boost::sml::aux::get_type_name<TGuard>()<< "  " << boost::sml::aux::get_type_name<TEvent>() << "  " << (result ? "[OK]" : "[Reject]");
    }

    /**
     * This function is called when an action is processed
     * @tparam SM the state machine
     * @tparam TAction the action
     * @tparam TEvent the event
     */
    template <class SM, class TAction, class TEvent>
    void log_action(const TAction&, const TEvent&)
    {
        //        printf("[%s][action] %s %s\n", sml::aux::get_type_name<SM>(), sml::aux::get_type_name<TAction>(),
        //               sml::aux::get_type_name<TEvent>());
//        LOG(INFO) << boost::sml::aux::get_type_name<SM>()<< "  " << boost::sml::aux::get_type_name<TAction>() << "  " <<boost::sml::aux::get_type_name<TEvent>();
    }

    /**
     * This function is called when a state change occurs
     *
     * @tparam SM the state machine
     * @tparam TSrcState the state type being exited
     * @tparam TDstState the state type being entered
     * @param src the state being exited
     * @param dst the state being entered
     */
    template <class SM, class TSrcState, class TDstState>
    void log_state_change(const TSrcState& src, const TDstState& dst)
    {
        // skip if state doesn't change
        if(src.c_str() == dst.c_str()){
            last_state_transition = "";
            return;
        }
        std::string message = "";
        message.append(boost::sml::aux::get_type_name<SM>());
        message.append(" ");
        message.append(src.c_str());
        message.append(" -> ");
        message.append(dst.c_str());
        last_state_transition = message; //boost::sml::aux::get_type_name<SM>() + " " + src.c_str() + " -> " + dst.c_str();
    }

    void flush_with_robot_id(unsigned int id) {
        if (id == robot_logs.begin()->first) {

            std::string output;
            for (const auto& [key, value] : robot_logs) {

                output.append(std::to_string(key));
                output.append(": ");
                output.append(value + "\n");
            }
            LOG(INFO) << output;
        }
       if (last_state_transition != "") {
        robot_logs[id] = last_state_transition;
      }
    }

protected:
    // Prevent construction and construction by copy, assignment, destruction.
    FSMLogger() = default;
    FSMLogger(const FSMLogger&) = default;
    FSMLogger& operator=(const FSMLogger&) = default;
    ~FSMLogger() = default;

private:
    std::map<unsigned int, std::string> robot_logs;
    std::string last_state_transition;
};


/**
 * The Tactic FSM framework uses the [SML library](https://github.com/boost-ext/sml), and
 * aims to create a readable style of FSM to implement tactic gameplay. See the MoveTactic
 * for an example of how to implement a tactic using this framework
 */

// An alias for an FSM
template <class T>
using FSM = boost::sml::sm<T, boost::sml::process_queue<std::queue>, boost::sml::logger<FSMLogger>>;

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
    const auto FUNCTION##_A =                                                            \
        [this](auto event, back::process<SUB_FSM::Update> processEvent)                  \
    { FUNCTION(event, processEvent); };

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
    state_machine.visit_current_states(
        [&name](const auto& state)
        {
            name = stripFSMState(TYPENAME(boost::sml::back::policies::get_state_name_t<
                                          std::decay_t<decltype(state)>>));
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
    state_machine.template visit_current_states<SSM>(
        [&name, &state_machine](const auto& state)
        {
            name = stripFSMState(TYPENAME(boost::sml::back::policies::get_state_name_t<
                                          std::decay_t<decltype(state)>>));
            using state_repr_t = std::decay_t<decltype(state)>;
            using state_t      = typename state_repr_t::type;
            if constexpr (is_sub_state_machine<state_t>::value)
            {
                using state_machine_t = typename state_machine_impl<state_t>::type;
                name += ".";
                name +=
                    getCurrentSubStateName<decltype(boost::sml::state<state_machine_t>)>(
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
    state_machine.visit_current_states(
        [&name, &state_machine](const auto& state)
        {
            name += getCurrentStateName(state_machine);
            using state_repr_t = std::decay_t<decltype(state)>;
            using state_t      = typename state_repr_t::type;
            if constexpr (is_sub_state_machine<state_t>::value)
            {
                using state_machine_t = typename state_machine_impl<state_t>::type;
                name += ".";
                name +=
                    getCurrentSubStateName<decltype(boost::sml::state<state_machine_t>)>(
                        state_machine);
            }
        });
    return name;
}
