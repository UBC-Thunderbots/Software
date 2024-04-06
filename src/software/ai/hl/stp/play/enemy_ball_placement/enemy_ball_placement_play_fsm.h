#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/world/game_state.h"

/**
 * This FSM implements the enemy ball placement play.
 * - If the ball placement position does not exist yet, then we wait
 * - Once the ball placement position exists, we set up 2 crease defenders and 3 robots to
 * stay near the ball without interfering
 *
 *                          placement point
 *                                +
 *
 *                     o    x  enemy robot with ball
 *        move robots    o
 *                         o
 *
 *   crease defenders  o   o
 *
 *        goalie         o
 *                    +-----+
 */

struct EnemyBallPlacementPlayFSM
{
    class WaitState;
    class AvoidState;

    struct ControlParams
    {
    };

    DEFINE_PLAY_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    /**
     * Creates an enemy ball placement play FSM
     *
     * @param ai_config the play config for this play FSM
     */
    explicit EnemyBallPlacementPlayFSM(TbotsProto::AiConfig ai_config);

    /**
     * Guard that checks if the ball placement point exists
     *
     * @param event the EnemyBallPlacementPlayFSM Update event
     */
    bool hasPlacementPoint(const Update& event);

    /**
     * Action that initializes the enemy ball placement behaviour
     *
     * @param event the EnemyBallPlacementPlayFSM Update event
     */
    void setPlacementPoint(const Update& event);

    /**
     * Action that positions robots to avoid ball placement interference
     *
     * @param event the EnemyBallPlacementPlayFSM Update event
     */
    void avoid(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(WaitState)
        DEFINE_SML_STATE(AvoidState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setPlacementPoint)
        DEFINE_SML_ACTION(avoid)

        DEFINE_SML_GUARD(hasPlacementPoint)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *WaitState_S + Update_E[hasPlacementPoint_G] / setPlacementPoint_A =
                AvoidState_S,
            WaitState_S + Update_E[!hasPlacementPoint_G] = WaitState_S,

            AvoidState_S + Update_E / avoid_A = AvoidState_S);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defenders;
    std::array<std::shared_ptr<MoveTactic>, 3> move_tactics;

    Point placement_point;
    double distance_to_keep;
};
