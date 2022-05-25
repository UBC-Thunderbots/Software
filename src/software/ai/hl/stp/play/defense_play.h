#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"

/**
 * The Defense Play tries to grab the ball from the enemy that has it, and all other
 * robots shadow the enemy robots in order of how threatening they are.
 */
class DefensePlay : public Play
{
   public:
    DefensePlay(TbotsProto::AiConfig config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

    const double ROBOT_SHADOWING_DISTANCE_METERS = ROBOT_MAX_RADIUS_METERS * 3;
};
