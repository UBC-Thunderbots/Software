#pragma once

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
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

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS


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
                                    const double &shadow_distance)
    {
        Vector enemy_to_shadowee_vector = ball_position - shadowee.position();

        return shadowee.position() + enemy_to_shadowee_vector.normalize(shadow_distance);
    }


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
                                    const Robot &shadowee, const double &shadow_distance)
    {
        std::vector<Robot> robots_to_ignore = {robot};
        if (friendlyTeam.goalie().has_value())
        {
            robots_to_ignore.emplace_back(friendlyTeam.goalie().value());
        }

        auto best_enemy_shot_opt =
            calcBestShotOnGoal(field, friendlyTeam, enemyTeam, shadowee.position(),
                               TeamType::FRIENDLY, robots_to_ignore);

        Vector enemy_shot_vector = field.friendlyGoalCenter() - shadowee.position();
        if (best_enemy_shot_opt)
        {
            enemy_shot_vector =
                best_enemy_shot_opt->getPointToShootAt() - shadowee.position();
        }
        return shadowee.position() + enemy_shot_vector.normalize(shadow_distance);
    }

    auto operator()()
    {
        using namespace boost::sml;

        const auto block_shot_s     = state<MoveFSM>;
        const auto block_pass_s     = state<BlockPassState>;
        const auto steal_and_chip_s = state<StealAndChipState>;

        const auto update_e = event<Update>;

        /**
         * Guard that checks if the enemy threat has ball
         *
         * @param event ShadowEnemyFSM::Update
         *
         * @return if the ball has been have_possession
         */
        const auto enemy_threat_has_ball = [](auto event) {
            std::optional<EnemyThreat> enemy_threat_opt =
                event.control_params.enemy_threat;
            if (enemy_threat_opt.has_value())
            {
                return enemy_threat_opt.value().has_ball;
            };
            LOG(WARNING) << "Enemy threat not initialized for robot "
                         << event.common.robot.id() << "\n";
            return false;
        };

        /**
         * Action to block the pass to our shadowee
         *
         *
         * @param event ShadowEnemyFSM::Update
         */
        const auto block_pass = [](auto event) {
            std::optional<EnemyThreat> enemy_threat_opt =
                event.control_params.enemy_threat;
            auto ball_position = event.common.world.ball().position();
            auto face_ball_orientation =
                (ball_position - event.common.robot.position()).orientation();

            // If no enemy_threat is found, the robot will default to blocking
            // the possible shot on net

            Point position_to_block =
                ball_position +
                (event.common.world.field().friendlyGoalCenter() - ball_position)
                    .normalize(event.control_params.shadow_distance);
            if (enemy_threat_opt.has_value())
            {
                position_to_block =
                    findBlockPassPoint(ball_position, enemy_threat_opt.value().robot,
                                       event.control_params.shadow_distance);
            };

            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), position_to_block, face_ball_orientation, 0,
                DribblerMode::OFF, BallCollisionType::AVOID,
                AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
        };

        /**
         * Action to block the shot from our shadowee
         *
         *
         * @param event ShadowEnemyFSM::Update
         */
        const auto block_shot = [this](auto event,
                                       back::process<MoveFSM::Update> processEvent) {
            std::optional<EnemyThreat> enemy_threat_opt =
                event.control_params.enemy_threat;
            auto ball_position = event.common.world.ball().position();
            auto face_ball_orientation =
                (ball_position - event.common.robot.position()).orientation();

            // If no enemy_threat is found, the robot will default to blocking
            // the possible shot on net

            Point position_to_block =
                ball_position +
                (event.common.world.field().friendlyGoalCenter() - ball_position)
                    .normalize(event.control_params.shadow_distance);
            if (enemy_threat_opt.has_value())
            {
                position_to_block = findBlockShotPoint(
                    event.common.robot, event.common.world.field(),
                    event.common.world.friendlyTeam(), event.common.world.enemyTeam(),
                    enemy_threat_opt.value().robot, event.control_params.shadow_distance);
            };

            MoveFSM::ControlParams control_params{
                .destination            = position_to_block,
                .final_orientation      = face_ball_orientation,
                .final_speed            = 0.0,
                .dribbler_mode          = DribblerMode::OFF,
                .ball_collision_type    = BallCollisionType::AVOID,
                .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                .max_allowed_speed_mode = MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                .target_spin_rev_per_s  = 0.0};

            processEvent(MoveFSM::Update(control_params, event.common));
        };

        /**
         * Action to steal and chip the ball
         *
         * Steal the ball if enemy threat is close enough and chip the ball away
         *
         * @param event ShadowEnemyFSM::Update
         */
        const auto steal_and_chip = [](auto event) {
            auto ball_position = event.common.world.ball().position();
            auto face_ball_orientation =
                (ball_position - event.common.robot.position()).orientation();
            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), ball_position, face_ball_orientation, 0,
                DribblerMode::MAX_FORCE, BallCollisionType::ALLOW,
                AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
        };


        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *block_shot_s + update_e[!enemy_threat_has_ball] / block_pass = block_pass_s,
            block_shot_s + update_e / block_shot, block_shot_s = steal_and_chip_s,
            block_pass_s + update_e[!enemy_threat_has_ball] / block_pass,
            block_pass_s + update_e[enemy_threat_has_ball] / block_shot = block_shot_s,
            steal_and_chip_s + update_e[enemy_threat_has_ball] / steal_and_chip,
            steal_and_chip_s + update_e[!enemy_threat_has_ball] / block_pass = X,
            X + update_e[!enemy_threat_has_ball] / block_pass = block_pass_s,
            X + update_e[enemy_threat_has_ball] / block_shot  = block_shot_s);
    }
};
