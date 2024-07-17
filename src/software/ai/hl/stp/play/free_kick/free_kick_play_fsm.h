#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/hl/stp/play/defense/defense_play.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/chip/chip_tactic.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/kick/kick_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/ai/hl/stp/tactic/receiver/receiver_tactic.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/ai/passing/pass_generator.h"
#include "software/ai/passing/receiver_position_generator.hpp"
#include "software/logger/logger.h"

/**
 * This FSM implements the free kick play. The logic of this play is:
 * - One robot (the kicker) attempts to shoot first. If there is a good shot, then it
 * will shoot the ball.
 * - If there is no good shot, the kicker will attempt to pass. Two robots try to get in
 * good receiving positions to receive a pass.
 * - If we cannot find a pass in time, the kicker will chip the ball towards the friendly
 * robot furthest up the field.
 */

struct FreeKickPlayFSM
{
    class SetupPositionState;
    class ShootState;
    class AttemptPassState;
    class PassState;
    class ChipState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates a free kick play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit FreeKickPlayFSM(const TbotsProto::AiConfig& ai_config);

    /**
     * Action that sets up the robots in position to perform the free kick
     *
     * @param event the FreeKickPlayFSM Update event
     */
    void setupPosition(const Update& event);

    /**
     * Guard that checks if the robots are in position to start the free kick
     *
     * @param event the FreeKickPlayFSM Update event
     *
     * @return whether the robots are in position to start the free kick
     */
    bool setupDone(const Update& event);

    /**
     * Updates the offensive positioning tactics
     *
     * @param world the world
     * @param num_tactics the number of tactics to assign
     * @param existing_receiver_positions A set of positions of existing receiver
     * positions that should be taken into account when assigning additional offensive
     * tactics.
     * @param pass_origin_override An optional point that the pass origin should be
     * overridden to
     */
    void updateReceiverPositioningTactics(
        const WorldPtr world, unsigned int num_tactics,
        const std::vector<Point>& existing_receiver_positions = {},
        const std::optional<Point>& pass_origin_override      = std::nullopt);

    /**
     * Sets the receiver and defender tactics for the free kick play
     *
     * @param tactics_to_run The priority tactic vector to add the tactics to
     * @param event The update event
     * @param ideal_num_receivers The ideal number of receivers to assign. This value
     * may be adjusted based on the number of available tactics.
     * @param num_tactics_already_assigned The number of tactics already assigned from
     * event.common.num_tactics
     * @param existing_receiver_positions A set of positions of existing receiver
     * positions that should be taken into account when assigning additional offensive
     * tactics.
     * @param pass_origin_override An optional point that the pass origin should be
     * overridden to
     */
    void setReceiverAndDefenderTactics(
        PriorityTacticVector& tactics_to_run, const Update& event,
        int ideal_num_receivers, int num_tactics_already_assigned,
        const std::vector<Point>& existing_receiver_positions = {},
        const std::optional<Point>& pass_origin_override      = std::nullopt);

    /**
     * Updates the kicker to align to the ball
     *
     * @param world the latest world
     */
    void updateAlignToBallTactic(const WorldPtr& world_ptr);

    /**
     * Action that starts the process of looking for a pass
     *
     * @param event the FreeKickPlayFSM Update event
     */
    void startLookingForPass(const Update& event);

    /**
     * Action that looks for a pass between the kicker and another robot
     *
     * @param event the FreeKickPlayFSM Update event
     */
    void lookForPass(const Update& event);

    /**
     * Action to shoot the ball towards the enemy goal directly
     *
     * @param event the FreeKickPlayFSM Update event
     */
    void shootBall(const Update& event);

    /**
     * Action to pass the ball to a friendly robot
     *
     * @param event the FreeKickPlayFSM Update event
     */
    void passBall(const Update& event);

    /**
     * Action to chip the ball towards the enemy net
     *
     * @param event the FreeKickPlayFSM Update event
     */
    void chipBall(const Update& event);

    /**
     * Guard that checks if we should shoot the ball towards the enemy goal
     *
     * @param event the FreeKickPlayFSM Update event
     *
     * @return whether we should shoot the ball directly
     */
    bool shotFound(const Update& event);

    /**
     * Guard that checks if we should pass the ball to a friendly robot
     *
     * @param event the FreeKickPlayFSM Update event
     *
     * @return whether we should pass the ball
     */
    bool passFound(const Update& event);

    /**
     * Guard on whether to abort the pass
     *
     * @param event the FreeKickPlayFSM Update event
     *
     * @return whether the pass should be aborted
     */
    bool shouldAbortPass(const Update& event);

    /**
     * Guard that checks if the allotted time to search for a pass is over
     *
     * @param event the FreeKickPlayFSM Update event
     *
     * @return whether the time to search for a pass is over
     */
    bool timeExpired(const Update& event);

    /**
     * Guard that checks if the shot is completed
     *
     * @param event the FreeKickPlayFSM Update event
     *
     * @return whether the shot is completed
     */
    bool shotDone(const Update& event);

    /**
     * Guard that checks if the pass is completed
     *
     * @param event the FreeKickPlayFSM Update event
     *
     * @return whether the pass is completed
     */
    bool passDone(const Update& event);

    /**
     * Guard that checks if the chip is completed
     *
     * @param event the FreeKickPlayFSM Update event
     *
     * @return whether the chip is completed
     */
    bool chipDone(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(SetupPositionState)
        DEFINE_SML_STATE(ShootState)
        DEFINE_SML_STATE(AttemptPassState)
        DEFINE_SML_STATE(PassState)
        DEFINE_SML_STATE(ChipState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setupPosition)
        DEFINE_SML_ACTION(shootBall)
        DEFINE_SML_ACTION(startLookingForPass)
        DEFINE_SML_ACTION(lookForPass)
        DEFINE_SML_ACTION(passBall)
        DEFINE_SML_ACTION(chipBall)

        DEFINE_SML_GUARD(setupDone)
        DEFINE_SML_GUARD(shotFound)
        DEFINE_SML_GUARD(shotDone)
        DEFINE_SML_GUARD(shouldAbortPass)
        DEFINE_SML_GUARD(passFound)
        DEFINE_SML_GUARD(passDone)
        DEFINE_SML_GUARD(chipDone)
        DEFINE_SML_GUARD(timeExpired)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            // Start with setting up the position of the kicker
            *SetupPositionState_S + Update_E[!setupDone_G] / setupPosition_A =
                SetupPositionState_S,

            // Shoot towards the enemy net directly if there is a clear shot
            SetupPositionState_S + Update_E[shotFound_G]       = ShootState_S,
            ShootState_S + Update_E[!shotDone_G] / shootBall_A = ShootState_S,
            ShootState_S + Update_E[shotDone_G]                = X,

            // Otherwise, start looking for a pass
            SetupPositionState_S + Update_E / startLookingForPass_A = AttemptPassState_S,

            // If the time to look for a pass is over, chip the ball towards the enemy net
            AttemptPassState_S + Update_E[timeExpired_G] = ChipState_S,

            // Keep looking for a pass
            AttemptPassState_S + Update_E[!passFound_G] / lookForPass_A =
                AttemptPassState_S,
            AttemptPassState_S + Update_E[passFound_G] = PassState_S,

            PassState_S + Update_E[shouldAbortPass_G]        = AttemptPassState_S,
            PassState_S + Update_E[!passDone_G] / passBall_A = PassState_S,
            PassState_S + Update_E[passDone_G]               = X,

            ChipState_S + Update_E[!chipDone_G] / chipBall_A = ChipState_S,
            ChipState_S + Update_E[chipDone_G]               = X);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::optional<Shot> shot;
    std::shared_ptr<MoveTactic> align_to_ball_tactic;
    std::shared_ptr<KickTactic> shoot_tactic;
    std::shared_ptr<ChipTactic> chip_tactic;
    std::shared_ptr<KickTactic> passer_tactic;
    std::shared_ptr<ReceiverTactic> receiver_tactic;
    std::vector<std::shared_ptr<MoveTactic>> receiver_positioning_tactics;
    std::shared_ptr<DefensePlay> defense_play;

    ReceiverPositionGenerator<EighteenZoneId> receiver_position_generator;
    PassGenerator pass_generator;
    PassWithRating best_pass_and_score_so_far;
    Timestamp pass_optimization_start_time;
};
