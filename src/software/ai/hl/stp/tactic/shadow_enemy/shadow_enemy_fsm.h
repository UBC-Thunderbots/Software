#pragma once

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "../tactic_base.hpp"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersects.h"
#include "software/logger/logger.h"


/**
 * The control parameters for updating ShadowEnemyFSM
 */
struct ShadowEnemyFSMControlParams
{
    // The Enemy Threat indicating which enemy to shadow
    std::optional<EnemyThreat> enemy_threat;

    // How far from the enemy the robot will position itself to shadow. If the enemy
    // threat has the ball, it will position itself to block the shot on goal.
    // Otherwise it will try to block the pass to the enemy threat.
    double shadow_distance;
};

struct ShadowEnemyFSM : TacticFSM<ShadowEnemyFSMControlParams>
{
   public:
    using Update = TacticFSM<ShadowEnemyFSMControlParams>::Update;
    class BlockPassState;
    class GoAndStealState;
    class StealAndPullState;

    /**
     * Constructor for ShadowEnemyFSM
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit ShadowEnemyFSM(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr)
        : TacticFSM<ShadowEnemyFSMControlParams>(ai_config_ptr)
    {
    }

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
     * Guard that checks if we may have contested the ball
     *
     * @param event ShadowEnemyFSM::Update
     *
     * @return if we are within dribbling range of ball
     */
    bool contestedBall(const Update &event);

    /**
     * Guard that checks if we have essentially blocked the shot
     *
     * @param event ShadowEnemyFSM::Update
     *
     * @return if we are blocking a shot
     */
    bool blockedShot(const Update &event);

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
     * Action to go to steal the ball
     *
     * Go to steal the ball if enemy threat is close enough and chip the ball away
     *
     * @param event ShadowEnemyFSM::Update
     */
    void goAndSteal(const Update &event);

    /**
     * Action to pull the ball
     *
     * Attempt to pull the ball away if within roller
     *
     * @param event ShadowEnemyFSM::Update
     */
    void stealAndPull(const Update &event);

    auto operator()()
    {
        using namespace boost::sml;

        DEFINE_SML_STATE(MoveFSM)
        DEFINE_SML_STATE(BlockPassState)
        DEFINE_SML_STATE(GoAndStealState)
        DEFINE_SML_STATE(StealAndPullState)

        DEFINE_SML_EVENT(Update)

        DEFINE_SML_GUARD(enemyThreatHasBall)
        DEFINE_SML_GUARD(contestedBall)
        DEFINE_SML_GUARD(blockedShot)

        DEFINE_SML_ACTION(blockPass)
        DEFINE_SML_ACTION(goAndSteal)
        DEFINE_SML_ACTION(stealAndPull)
        DEFINE_SML_SUB_FSM_UPDATE_ACTION(blockShot, MoveFSM)

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *MoveFSM_S + Update_E[!enemyThreatHasBall_G] / blockPass_A = BlockPassState_S,
            MoveFSM_S + Update_E[blockedShot_G] / goAndSteal_A = GoAndStealState_S,
            MoveFSM_S + Update_E / blockShot_A, MoveFSM_S = GoAndStealState_S,
            BlockPassState_S + Update_E[!enemyThreatHasBall_G] / blockPass_A,
            BlockPassState_S + Update_E[enemyThreatHasBall_G] / blockShot_A = MoveFSM_S,
            GoAndStealState_S +
                Update_E[enemyThreatHasBall_G && !contestedBall_G] / goAndSteal_A,
            GoAndStealState_S + Update_E[enemyThreatHasBall_G && contestedBall_G] /
                                    goAndSteal_A = StealAndPullState_S,
            GoAndStealState_S + Update_E[!enemyThreatHasBall_G] / blockPass_A = X,
            StealAndPullState_S + Update_E[enemyThreatHasBall_G] / stealAndPull_A,
            StealAndPullState_S + Update_E[!enemyThreatHasBall_G] / blockPass_A = X,
            X + Update_E[!enemyThreatHasBall_G] / blockPass_A = BlockPassState_S,
            X + Update_E[enemyThreatHasBall_G] / blockShot_A  = MoveFSM_S,
            X + Update_E / SET_STOP_PRIMITIVE_ACTION          = X);
    }

   private:
    // Here we define roughly how close the enemy needs to be to the ball to be considered
    // as possessing the ball, this was determined experimentally in the simulator where
    // it was determined that vision often mistake the ball to be further away from the
    // enemy than it actually is, leading the robot to overcommit to stealing the ball
    static constexpr double ENEMY_NEAR_BALL_DIST_M = 0.22;

    // This is just checking whether it the defender is within a reasonable distance
    // to start pressing the robot
    static constexpr double NEAR_PRESS_M = 0.8;

    // Angle that enemy has to be facing the net within to consider going for the ball
    static constexpr double ENEMY_FACE_RADIANS = M_PI / 4.0;
};
