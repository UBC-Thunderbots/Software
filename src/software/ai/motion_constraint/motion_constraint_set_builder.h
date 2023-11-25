#pragma once

#include <set>

#include "proto/primitive.pb.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/hl/stp/tactic/tactic_visitor.h"
#include "software/world/game_state.h"


/**
 * Builds motion constraints based on game_state and tactic
 *
 * @param game_state Current GameState to process
 * @param tactic Current Tactic to process
 *
 * @return set of MotionConstraints
 */
std::set<TbotsProto::MotionConstraint> buildMotionConstraintSet(
    const GameState &game_state, const Tactic &tactic);



/**
 * Builds a set of motion constraints by adding motion constraints determined from
 * game state
 *
 * @param game_state GameState to generate move constraints from
 *
 * @return set of MotionConstraints
 */
std::set<TbotsProto::MotionConstraint> buildMotionConstraintSetFromGameState(
    const GameState &game_state);
