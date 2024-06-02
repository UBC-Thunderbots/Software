#pragma once

#include "proto/parameters.pb.h"
#include "shared/constants.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/hl/stp/tactic/crease_defender/crease_defender_tactic.h"
#include "software/ai/hl/stp/tactic/goalie/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/stadium.h"
#include "software/world/game_state.h"

/**
 * This FSM implements the enemy ball placement play.
 * - If the ball placement point does not exist yet, then we wait
 * - If the ball placement point exists, but the ball is far from the placement point,
 * then we focus on avoiding ball placement interference. If our robot R is in the ball
 * placement stadium, then it will move perpendicular to the segment connecting * and +,
 * to either p1 or p2, whichever one is within the field boundaries and closest to R
 *
 *     /‾‾‾‾‾\
 *     |  +  |        placement point
 *     |     |
 *     |     |
 *  p1 |  R  | p2
 *     |     |
 *     |  *  |        ball
 *     \_____/
 *
 * - Once the ball is near the placement point, we set up 1 goalie, 2 crease defenders and
 * 3 robots to stay near the ball without interfering
 *
 *            +
 *      R   x  enemy robot with ball
 *        R
 *          R
 *
 *      D   D
 *
 *        G
 *     +-----+
 */

struct EnemyBallPlacementPlayFSM
{
    class WaitState;
    class AvoidState;
    class DefenseState;

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
     * Guard that checks if the ball is nearly placed
     *
     * @param event the EnemyBallPlacementPlayFSM Update event
     */
    bool isNearlyPlaced(const Update& event);

    /**
     * Action that positions robots to avoid ball placement interference
     *
     * @param event the EnemyBallPlacementPlayFSM Update event
     */
    void avoid(const Update& event);

    /**
     * Action that positions robots in a defensive formation
     *
     * @param event the EnemyBallPlacementPlayFSM Update event
     */
    void enterDefensiveFormation(const Update& event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(WaitState)
        DEFINE_SML_STATE(AvoidState)
        DEFINE_SML_STATE(DefenseState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_ACTION(setPlacementPoint)
        DEFINE_SML_ACTION(avoid)
        DEFINE_SML_ACTION(enterDefensiveFormation)

        DEFINE_SML_GUARD(hasPlacementPoint)
        DEFINE_SML_GUARD(isNearlyPlaced)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *WaitState_S + Update_E[hasPlacementPoint_G] / setPlacementPoint_A =
                AvoidState_S,
            WaitState_S + Update_E[!hasPlacementPoint_G] = WaitState_S,

            AvoidState_S + Update_E[!isNearlyPlaced_G] / avoid_A = AvoidState_S,
            AvoidState_S + Update_E[isNearlyPlaced_G]            = DefenseState_S,

            DefenseState_S + Update_E[isNearlyPlaced_G] / enterDefensiveFormation_A =
                DefenseState_S,
            DefenseState_S + Update_E[!isNearlyPlaced_G] = AvoidState_S);
    }

   private:
    TbotsProto::AiConfig ai_config;
    std::array<std::shared_ptr<CreaseDefenderTactic>, 2> crease_defender_tactics;
    std::array<std::shared_ptr<AvoidInterferenceTactic>, 6> avoid_interference_tactics;
    std::array<std::shared_ptr<MoveTactic>, 3> move_tactics;
    std::shared_ptr<GoalieTactic> goalie_tactic;

    Point placement_point;
    double distance_to_keep_meters;
    double nearly_placed_threshold_meters;  // Threshold for the ball to be considered
                                            // nearly placed
};
