#pragma once

#include <functional>
#include <include/boost/sml.hpp>
#include <queue>
#include <type_traits>

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
 * Unimplemented stub. Doesn't expose type so that use will throw an error.
 *
 * @tparam T Any type
 */
template <typename T>
struct SMLCallbackTraits;

/**
 * Specializes against callback functions meant for Boost::SML and exposes their trait
 * typenames The three template values make up the declaration of a function. const
 * variant below
 *
 * @tparam FSMClass The FSM class the function belongs to
 * @tparam Ret The return value
 * @tparam Event The type of event being processed.
 */
template <typename FSMClass, typename Ret, typename Event>
struct SMLCallbackTraits<Ret (FSMClass::*)(const Event&)>
{
    using FSMType    = FSMClass;
    using EventType  = Event;
    using ReturnType = Ret;
};
template <typename FSMClass, typename Ret, typename Event>
struct SMLCallbackTraits<Ret (FSMClass::*)(const Event&) const>
{
    using FSMType    = FSMClass;
    using EventType  = Event;
    using ReturnType = Ret;
};

/**
 * Callable wrapper around FSM member function that can be used as an SML guard.
 *
 * @tparam GuardFn The function to turn into a guard.
 */
template <auto GuardFn>
class SMLGuard
{
    using Traits = SMLCallbackTraits<decltype(GuardFn)>;
    static_assert(std::is_same_v<typename Traits::ReturnType, bool>,
                  "an SML guard must return bool");

   public:
    explicit SMLGuard(Traits::FSMType* fsm) : fsm_(fsm) {}
    bool operator()(const Traits::EventType& event) const
    {
        return (fsm_->*GuardFn)(event);
    }

   private:
    Traits::FSMType* fsm_;
};

/**
 * Callable wrapper around FSM member function that can be used as an SML action.
 *
 * @tparam ActionFn The function to turn into an action.
 */
template <auto ActionFn>
class SMLAction
{
    using Traits = SMLCallbackTraits<decltype(ActionFn)>;
    static_assert(std::is_void_v<typename Traits::ReturnType>,
                  "an SML action must return void");

   public:
    explicit SMLAction(Traits::FSMType* fsm) : fsm_(fsm) {}
    void operator()(const Traits::EventType& event) const
    {
        (fsm_->*ActionFn)(event);
    }

   private:
    Traits::FSMType* fsm_;
};

/**
 * Specializes against callback functions meant for Boost::SML and exposes their trait
 * typenames The three template values make up the declaration of a function. In
 * particular, this class is for actions that utilize subFSMs.
 *
 * @tparam FSMClass The FSM class the function belongs to.
 * @tparam Event The type of event from the FSM that must be processed.
 * @tparam SubEvent The type of event from the subFSM that must be processed.
 */
template <typename FSMClass, typename Event, typename SubEvent>
struct SMLCallbackTraits<void (FSMClass::*)(const Event&,
                                            boost::sml::back::process<SubEvent>)>
{
    using FSMType      = FSMClass;
    using EventType    = Event;
    using SubEventType = SubEvent;
    using ReturnType   = void;
};

/**
 * Callable wrapper around FSM member function that can be used as an SML action for
 * updating a sub fsm
 * @tparam ActionFn The function to turn into an subFSM update action.
 */
template <auto ActionFn>
class SMLSubFSMUpdateAction
{
    using Traits = SMLCallbackTraits<decltype(ActionFn)>;

   public:
    explicit SMLSubFSMUpdateAction(Traits::FSMType* fsm) : fsm_(fsm) {}

    void operator()(
        const Traits::EventType& event,
        boost::sml::back::process<typename Traits::SubEventType> processEvent) const
    {
        (fsm_->*ActionFn)(event, processEvent);
    }

   private:
    Traits::FSMType* fsm_;
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
