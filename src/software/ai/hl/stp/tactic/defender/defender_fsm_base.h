#pragma once

#include "proto/tactic.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/dribble/dribble_fsm.h"
#include "software/ai/hl/stp/tactic/tactic_base.h"
#include "software/ai/hl/stp/tactic/tactic_fsm.h"
#include "software/logger/logger.h"

struct DefenderFSMBase
{
    /**
     * Guard that determines whether it is appropriate to steal the ball for FSMs
     *
     * @param world_ptr The current state of the world
     * @param robot The current defender robot
     * @param ball_steal_mode The aggressive behaviour of the defender denoting whether to
     * steal the ball
     * @param defender_steal_config The dynamic params containing the defender's config
     *
     * @return true if stealing is enabled and the ball is nearby, unguarded by the enemy,
     *          and within a max get possession threshold
     */
    static bool ballNearbyWithoutThreat(
        const WorldPtr& world_ptr, const Robot& robot,
        const TbotsProto::BallStealMode& ball_steal_mode,
        const TbotsProto::DefenderStealConfig& defender_steal_config);
    /**
     * This is the Action that prepares for getting possession of the ball with FSMs
     * @param tactic_update the Defender's Update event
     * @param processEvent processes the DribbleFSM::Update
     */
    static void prepareGetPossession(
        const TacticUpdate& tactic_update,
        boost::sml::back::process<DribbleFSM::Update> processEvent);
};
