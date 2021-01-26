#pragma once

#include <include/boost/sml.hpp>
#include <queue>

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/transition_conditions.h"

// alias for FSMs that have at least 2 levels of hierarchy
template <class FSM>
using HFSM = boost::sml::sm<FSM, boost::sml::process_queue<std::queue>>;

// alias for FSMs that have no hierarchy
template <class FSM>
using BaseFSM = boost::sml::sm<FSM>;
