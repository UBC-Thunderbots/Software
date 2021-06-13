#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/passing/pass.h"
#include "software/ai/evaluation/find_open_areas.h"

MAKE_ENUM(AttackerFSMStates,
        KEEP_AWAY,
        SHOOTING,
        PASSING
        )

struct PasserFSM
{
    using ControlParams = PivotKickFSM::ControlParams;
    // this struct defines the only event that the PivotKickFSM responds to
    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()() {
        using namespace boost::sml;

        const auto pass_s = state<PivotKickFSM>;
        return make_transition_table(
                *pass_s = X
                        );
    }
};

struct ShooterFSM
{
    using ControlParams = PivotKickFSM::ControlParams;
    // this struct defines the only event that the PivotKickFSM responds to
    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()() {
        using namespace boost::sml;

        const auto shoot_s = state<PivotKickFSM>;
        return make_transition_table(
                *shoot_s = X
        );
    }
};

struct AttackerFSM
{
    struct ControlParams
    {
        // The pass to execute
        std::optional<Pass> pass = std::nullopt;
        // The shot to take
        std::optional<Shot> shot = std::nullopt;
        // The point the robot will chip towards if it is unable to shoot and is in danger
        // of losing the ball to an enemy
        std::optional<Point> chip_target;
        // shoot goal config
        std::shared_ptr<const AttackerTacticConfig> attacker_tactic_config;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    AttackerFSMStates _current_state;

    auto operator()()
    {
        using namespace boost::sml;

//        const auto pivot_kick_s = state<PivotKickFSM>;
        const auto shoot_s = state<ShooterFSM>;
        const auto pass_s = state<PasserFSM>;
        const auto keep_away_s  = state<DribbleFSM>;
        const auto update_e     = event<Update>;

        /**
         * Action that updates the PivotKickFSM to shoot or pass
         *
         * @param event AttackerFSM::Update event
         * @param processEvent processes the PivotKickFSM::Update
         */
//        const auto pivot_kick = [](auto event,
//                                   back::process<PivotKickFSM::Update> processEvent) {
//            auto ball_position = event.common.world.ball().position();
////            Point chip_target  = event.common.world.field().enemyGoalCenter();
////            if (event.control_params.chip_target)
////            {
////                chip_target = event.control_params.chip_target.value();
////            }
////            // default to chipping the ball away
////            PivotKickFSM::ControlParams control_params{
////                .kick_origin    = ball_position,
////                .kick_direction = (chip_target - ball_position).orientation(),
////                .auto_chip_or_kick =
////                    AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP,
////                                   (chip_target - ball_position).length()}};
//
//            if (event.control_params.shot)
//            {
//                // shoot on net
//            PivotKickFSM::ControlParams control_params{
//                    .kick_origin = ball_position,
//                    .kick_direction =
//                        (event.control_params.shot->getPointToShootAt() - ball_position)
//                            .orientation(),
//                    .auto_chip_or_kick =
//                        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
//                                       BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}};
//                processEvent(PivotKickFSM::Update(control_params, event.common));
//            }
//            else if (event.control_params.pass)
//            {
//                PivotKickFSM::ControlParams control_params{
//                    .kick_origin    = event.control_params.pass->passerPoint(),
//                    .kick_direction = event.control_params.pass->passerOrientation(),
//                    .auto_chip_or_kick =
//                        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
//                                       event.control_params.pass->speed()}};
//                processEvent(PivotKickFSM::Update(control_params, event.common));
//            }
//        };
//
        const auto shoot = [&](auto event,
                                   back::process<PivotKickFSM::Update> processEvent) {
                // shoot on net
            _current_state = AttackerFSMStates::PASSING;
//            std::cout << "in shoot" << std::endl;
            auto ball_position = event.common.world.ball().position();
                PivotKickFSM::ControlParams control_params{
                        .kick_origin = ball_position,
                        .kick_direction =
                        (event.control_params.shot->getPointToShootAt() - ball_position)
                                .orientation(),
                        .auto_chip_or_kick =
                        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                       BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}};
                processEvent(PivotKickFSM::Update(control_params, event.common));
        };

        const auto pass = [&](auto event,
                                   back::process<PivotKickFSM::Update> processEvent) {
            _current_state = AttackerFSMStates::PASSING;
//            std::cout << "in pass" << std::endl;
                PivotKickFSM::ControlParams control_params{
                        .kick_origin    = event.control_params.pass->passerPoint(),
                        .kick_direction = event.control_params.pass->passerOrientation(),
                        .auto_chip_or_kick =
                        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                       event.control_params.pass->speed()}};
                processEvent(PivotKickFSM::Update(control_params, event.common));
        };

        /**
         * Action that updates the DribbleFSM to keep the ball away
         *
         * @param event AttackerFSM::Update event
         * @param processEvent processes the DribbleFSM::Update
         */
        const auto keep_away = [&](auto event,
                                  back::process<DribbleFSM::Update> processEvent) {
//            _current_state = AttackerFSMStates::KEEP_AWAY;
//            std::cout << "in keep away" << std::endl;
            // TODO (#2073): Implement a more effective keep away tactic
//            DribbleFSM::ControlParams control_params{
//                .dribble_destination       = std::nullopt,
//                .final_dribble_orientation = std::nullopt,
//                .allow_excessive_dribbling = true};
//            processEvent(DribbleFSM::Update(control_params, event.common));

// Face away from the closest enemy
            std::vector<Robot> close_enemies;
            for(auto& robot : event.common.world.enemyTeam().getAllRobots()) {
                auto dist = [&](auto& enemy) {
                    return (event.common.robot.position() - enemy.position()).length();
                };
                if (dist(robot) < 0.5) {
                    close_enemies.emplace_back(robot);
                }
            }
            std::optional<Angle> orientation = std::nullopt;
            auto good_open_circles = findGoodChipTargets(event.common.world);
            std::optional<Point> dest = std::nullopt;
            if(good_open_circles.size() > 0) {
                dest = good_open_circles[0].origin();
            }
            if (close_enemies.size() > 0) {
                // Choose an orientation that tries to point away from close enemies
                Vector sum(0,0);
                for(auto& robot : close_enemies) {
                    sum +=event.common.robot.position() - robot.position();
                }
                orientation = (sum / static_cast<double>(close_enemies.size())).orientation();
            }
            DribbleFSM::ControlParams control_params{
                    .dribble_destination       = std::nullopt,
                    .final_dribble_orientation = orientation,
                    .allow_excessive_dribbling = false};
            processEvent(DribbleFSM::Update(control_params, event.common));
        };

        /**
         * Guard that checks if the ball should be kicked, which is when there's a nearby
         * enemy or a good pass/shot
         *
         * @param event AttackerFSM::Update event
         *
         * @return if the ball should be kicked
         */
//        const auto should_kick = [](auto event) {
//            Point robot_left_side = event.common.robot.position() +
//                                    Vector::createFromAngle(event.common.robot.orientation()).perpendicular().normalize(
//                                            ROBOT_MAX_RADIUS_METERS);
//            Point robot_right_side = event.common.robot.position() + Vector::createFromAngle(
//                    event.common.robot.orientation()).perpendicular().normalize(ROBOT_MAX_RADIUS_METERS);
//            Vector stealing_zone = Vector::createFromAngle(event.common.robot.orientation()).normalize(
//                    event.control_params.attacker_tactic_config->getEnemyAboutToStealBallDistance()->value());
//            // check for enemy threat
//            Polygon about_to_steal_danger_zone({
//                                                       robot_left_side,
//                                                       robot_left_side + stealing_zone,
//                                                       robot_right_side + stealing_zone,
//                                                       robot_right_side
//                                               });
//            for (const auto &enemy : event.common.world.enemyTeam().getAllRobots()) {
//                if (contains(about_to_steal_danger_zone, enemy.position())) {
//                    return true;
//                }
//            }
//            // otherwise check for shot or pass
//            return event.control_params.pass || event.control_params.shot;
//        }

        const auto has_shot = [](auto event) {
            auto ret =  event.control_params.shot.has_value();
//            std::cout << "has shot: " << ret << std::endl;
            return ret;
        };

        const auto has_pass = [](auto event) {
            auto ret =  event.control_params.pass.has_value();
//            std::cout << "has pass: " << ret << std::endl;
            return ret;
        };

        return make_transition_table(
            // clang-format off
            // src_state + event [guard] / action = dest_state
            *keep_away_s + update_e[has_shot] / shoot = shoot_s,
            keep_away_s + update_e[has_pass] / pass = pass_s,
            keep_away_s + update_e[!(has_shot || has_pass)] / keep_away = keep_away_s,
            shoot_s + update_e[has_shot] / shoot,
            shoot_s + update_e[!has_shot && has_pass] / pass = pass_s,
            shoot_s + update_e[!(has_shot || has_pass)] / keep_away = keep_away_s,
////            shoot_s + update_e[!has_shot] / keep_away = keep_away_s,
            pass_s + update_e[has_pass] / pass,
            pass_s + update_e[!has_pass && has_shot] / shoot = shoot_s,
            pass_s + update_e[!(has_pass || has_shot)] / keep_away = keep_away_s,
            shoot_s = X,
//            X + update_e /  = X
            pass_s = X
            );
            // clang-format on
    }
};
