#pragma once

#include "software/ai/evaluation/calc_best_shot.h"
#include "software/ai/evaluation/enemy_threat.h"
#include "software/ai/hl/stp/tactic/move/move_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/geom/algorithms/distance.h"

struct ShadowEnemyFSM
{
    class BlockState;
    class StealAndChipState;

    // this struct defines the unique control parameters that the ShadowEnemyFSM requires
    // in its update
    struct ControlParams
    {
        // The Enemy Threat indicating which enemy to shadow
        std::optional<EnemyThreat> enemy_threat;

        // How far from the enemy the robot will position itself to shadow
        double shadow_distance;

        // How close to the enemy before the robot will try to steal and chip the ball
        double steal_and_chip_distance;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS


    // Distance to chip the ball when trying to yeet it
    // TODO (#1878): Replace this with a more intelligent chip distance system
    static constexpr double YEET_CHIP_DISTANCE_METERS = 2.0;

    auto operator()()
    {
        using namespace boost::sml;

        const auto block_s          = state<BlockState>;
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
            return false;
        };

        /**
         * Guard that checks if the enemy threat is near
         * our robot
         *
         * @param event ShadowEnemyFSM::Update
         *
         * @return if the ball has been have_possession
         */
        const auto enemy_threat_is_near = [](auto event) {
            std::optional<EnemyThreat> enemy_threat_opt =
                event.control_params.enemy_threat;
            if (enemy_threat_opt.has_value())
            {
                return distance(enemy_threat_opt.value().robot.position(),
                                event.common.robot.position()) <=
                       event.control_params.steal_and_chip_distance;
            };
            return false;
        };

        /**
         * Action to block the ball
         *
         * If the enemy can pass to the robot we are shadowing, block the pass.
         * Else, block the net
         *
         * @param event ShadowEnemyFSM::Update
         */
        const auto block_pass_or_net = [](auto event) {
            auto ball_position = event.common.world.ball().position();
            auto face_ball_orientation =
                (ball_position - event.common.robot.position()).orientation();
            Point position_to_block;
            if (!event.control_params.enemy_threat.value().has_ball)
            {
                Vector enemy_to_passer_vector =
                    event.control_params.enemy_threat.value().passer.value().position() -
                    event.control_params.enemy_threat.value().robot.position();

                position_to_block =
                    event.control_params.enemy_threat.value().robot.position() +
                    enemy_to_passer_vector.normalize(
                        event.control_params.shadow_distance);
            }
            else
            {
                std::vector<Robot> robots_to_ignore = {event.common.robot};
                if (event.common.world.friendlyTeam().goalie().has_value())
                {
                    robots_to_ignore.emplace_back(
                        event.common.world.friendlyTeam().goalie().value());
                }

                auto best_enemy_shot_opt = calcBestShotOnGoal(
                    event.common.world.field(), event.common.world.friendlyTeam(),
                    event.common.world.enemyTeam(),
                    event.control_params.enemy_threat.value().robot.position(),
                    TeamType::FRIENDLY, robots_to_ignore);

                Vector enemy_shot_vector = Vector(0, 0);
                if (best_enemy_shot_opt)
                {
                    enemy_shot_vector =
                        best_enemy_shot_opt->getPointToShootAt() -
                        event.control_params.enemy_threat.value().robot.position();
                }
                else
                {
                    enemy_shot_vector =
                        event.common.world.field().friendlyGoalCenter() -
                        event.control_params.enemy_threat.value().robot.position();
                }

                position_to_block =
                    event.control_params.enemy_threat.value().robot.position() +
                    enemy_shot_vector.normalize(event.control_params.shadow_distance);
            }

            event.common.set_intent(std::make_unique<MoveIntent>(
                event.common.robot.id(), position_to_block, face_ball_orientation, 0,
                DribblerMode::OFF, BallCollisionType::AVOID,
                AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
                MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0));
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
            *block_s + update_e[!enemy_threat_has_ball] / block_pass_or_net,
            block_s + update_e[!enemy_threat_is_near] / block_pass_or_net,
            block_s + update_e[enemy_threat_is_near] / steal_and_chip = steal_and_chip_s,
            steal_and_chip_s + update_e[enemy_threat_has_ball] / steal_and_chip,
            steal_and_chip_s + update_e[!enemy_threat_has_ball] / block_pass_or_net = X,
            X + update_e[!enemy_threat_has_ball] / block_pass_or_net = block_s,
            X + update_e[!enemy_threat_is_near] / block_pass_or_net  = block_s,
            X + update_e[enemy_threat_is_near] / steal_and_chip      = steal_and_chip_s

        );
    }
};
