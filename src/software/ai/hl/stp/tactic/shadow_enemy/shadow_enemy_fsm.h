#pragma once

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/algorithms/distance.h"
#include "software/logger/logger.h"

struct ShadowEnemyFSM
{
   public:
    class BlockPassState;
    class StealAndChipState;

    // this struct defines the unique control parameters that the ShadowEnemyFSM requires
    // in its update
    struct ControlParams
    {
        // The Enemy Threat indicating which enemy to shadow
        std::optional<EnemyThreat> enemy_threat;

        // How far from the enemy the robot will position itself to shadow. If the enemy
        // threat has the ball, it will position itself to block the shot on goal.
        // Otherwise it will try to block the pass to the enemy threat.
        double shadow_distance;
    };

    DEFINE_TACTIC_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS


    // Distance to chip the ball when trying to yeet it
    // TODO (#1878): Replace this with a more intelligent chip distance system
    static constexpr double YEET_CHIP_DISTANCE_METERS = 2.0;


    /**
     * Calculates the point to block the pass to the robot we are shadowing
     *
     * @param ball_position The position of the ball
     * @param shadowee The enemy robot we are shadowing
     * @param shadow_distance The distance our friendly robot will position itself away
     * from the shadowee
     */
    static Point findBlockPassPoint(const Point &ball_position, const Robot &shadowee,
                                    const double &shadow_distance);

    /**
     * Calculates the point to block the shot from the robot we are shadowing
     *
     * @param robot The robot that is shadowing
     * @param field The field to shadow on
     * @param friendlyTeam The friendly team
     * @param enemyTeam The enemy team
     * @param shadowee The enemy robot we are shadowing
     * @param shadow_distance The distance our friendly robot will position itself away
     * from the shadowee
     */
    static Point findBlockShotPoint(const Robot &robot, const Field &field,
                                    const Team &friendlyTeam, const Team &enemyTeam,
                                    const Robot &shadowee, const double &shadow_distance);

    /**
     * Guard that checks if the enemy threat has ball
     *
     * @param event ShadowEnemyFSM::Update
     *
     * @return if the ball has been have_possession
     */
    bool enemyThreatHasBall(const Update &event);

    /**
     * Action to block the pass to our shadowee
     *
     *
     * @param event ShadowEnemyFSM::Update
     */
    void blockPass(const Update &event);

    /**
     * Action to block the shot from our shadowee
     *
     *
     * @param event ShadowEnemyFSM::Update
     */
    void blockShot(const Update &event,
                   boost::sml::back::process<MoveFSM::Update> processEvent);

    /**
     * Action to steal and chip the ball
     *
     * Steal the ball if enemy threat is close enough and chip the ball away
     *
     * @param event ShadowEnemyFSM::Update
     */
    void stealAndChip(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(MoveFSM)
        DEFINE_SML_STATE(BlockPassState)
        DEFINE_SML_STATE(StealAndChipState)
        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(enemyThreatHasBall)
        DEFINE_SML_ACTION(blockPass)
        DEFINE_SML_ACTION(stealAndChip)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(blockShot, MoveFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *MoveFSM_S + Update_E[!enemyThreatHasBall_G] / stealAndChip_A =
                StealAndChipState_S,
            MoveFSM_S + Update_E / blockShot_A, MoveFSM_S = StealAndChipState_S,
            BlockPassState_S + Update_E[!enemyThreatHasBall_G] / blockPass_A,
            BlockPassState_S + Update_E[enemyThreatHasBall_G] / blockShot_A = MoveFSM_S,
            StealAndChipState_S + Update_E[!enemyThreatHasBall_G] / stealAndChip_A,
            StealAndChipState_S + Update_E[enemyThreatHasBall_G] / blockPass_A = X,
            X + Update_E[!enemyThreatHasBall_G] / blockPass_A = BlockPassState_S,
            X + Update_E[enemyThreatHasBall_G] / blockShot_A  = MoveFSM_S,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION          = X);
    }
};
