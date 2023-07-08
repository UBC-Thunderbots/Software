#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_fsm.h"

Point ShadowEnemyFSM::findBlockPassPoint(const Point &ball_position,
                                         const Robot &shadowee,
                                         const double &shadow_distance)
{
    Vector enemy_to_shadowee_vector = ball_position - shadowee.position();

    return shadowee.position() + enemy_to_shadowee_vector.normalize(shadow_distance);
}

Point ShadowEnemyFSM::findBlockShotPoint(const Robot &robot, const Field &field,
                                         const Team &friendlyTeam, const Team &enemyTeam,
                                         const Robot &shadowee,
                                         const double &shadow_distance)
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
            best_enemy_shot_opt.value().getPointToShootAt() - shadowee.position();
    }
    return shadowee.position() + enemy_shot_vector.normalize(shadow_distance);
}

bool ShadowEnemyFSM::enemyThreatHasBall(const Update &event)
{
    std::optional<EnemyThreat> enemy_threat_opt = event.control_params.enemy_threat;
    if (enemy_threat_opt.has_value())
    {
        return enemy_threat_opt.value().has_ball;
    };
    LOG(WARNING) << "Enemy threat not initialized for robot " << event.common.robot.id()
                 << "\n";
    return false;
}

void ShadowEnemyFSM::blockPass(const Update &event)
{
    std::optional<EnemyThreat> enemy_threat_opt = event.control_params.enemy_threat;
    auto ball_position                          = event.common.world.ball().position();
    auto orientation_to_face =
        (ball_position - event.common.robot.position()).orientation();

    // If no enemy_threat is found, the robot will default to blocking
    // the possible shot on net

    Point position_to_block =
        ball_position + (event.common.world.field().friendlyGoalCenter() - ball_position)
                            .normalize(event.control_params.shadow_distance);
    if (enemy_threat_opt.has_value())
    {
        position_to_block =
            findBlockPassPoint(ball_position, enemy_threat_opt.value().robot,
                               event.control_params.shadow_distance);
        orientation_to_face = (enemy_threat_opt.value().robot.position() - event.common.robot.position()).orientation();
    };

    event.common.set_primitive(createMovePrimitive(
            CREATE_MOTION_CONTROL(position_to_block), orientation_to_face, 0, false,
            TbotsProto::DribblerMode::INDEFINITE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
            event.common.robot.robotConstants()));
}

void ShadowEnemyFSM::blockShot(const Update &event,
                               boost::sml::back::process<MoveFSM::Update> processEvent)
{
    std::optional<EnemyThreat> enemy_threat_opt = event.control_params.enemy_threat;
    auto ball_position                          = event.common.world.ball().position();
    auto orientation_to_face =
        (ball_position - event.common.robot.position()).orientation();

    // If no enemy_threat is found, the robot will default to blocking
    // the possible shot on net

    Point position_to_block =
        ball_position + (event.common.world.field().friendlyGoalCenter() - ball_position)
                            .normalize(event.control_params.shadow_distance);
    if (enemy_threat_opt.has_value())
    {
        position_to_block = findBlockShotPoint(
            event.common.robot, event.common.world.field(),
            event.common.world.friendlyTeam(), event.common.world.enemyTeam(),
            enemy_threat_opt.value().robot, event.control_params.shadow_distance);

        orientation_to_face = (enemy_threat_opt.value().robot.position() - event.common.robot.position()).orientation();
    };

    MoveFSM::ControlParams control_params{
        .destination            = position_to_block,
        .final_orientation      = orientation_to_face,
        .final_speed            = 0.0,
        .dribbler_mode          = TbotsProto::DribblerMode::INDEFINITE,
        .ball_collision_type    = TbotsProto::BallCollisionType::ALLOW,
        .auto_chip_or_kick      = AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        .max_allowed_speed_mode = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        .target_spin_rev_per_s  = 0.0};

    processEvent(MoveFSM::Update(control_params, event.common));
}

void ShadowEnemyFSM::steal(const Update &event)
{
    auto ball_position = event.common.world.ball().position();
    auto face_ball_orientation =
            (ball_position - event.common.robot.position()).orientation();

    event.common.set_primitive(createMovePrimitive(
            CREATE_MOTION_CONTROL(ball_position), face_ball_orientation, 0, false,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
            event.common.robot.robotConstants(), std::optional<double>()));
}

void ShadowEnemyFSM::stealAndChip(const Update &event)
{
    auto ball_position = event.common.world.ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();

    event.common.set_primitive(createMovePrimitive(
        CREATE_MOTION_CONTROL(ball_position), face_ball_orientation, 0, false,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP, YEET_CHIP_DISTANCE_METERS},
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT, 0.0,
        event.common.robot.robotConstants(), std::optional<double>()));
}

bool ShadowEnemyFSM::isGoodToChip(const Update &event)
{
    Angle robot_orientation         = event.common.robot.orientation();
    Vector direction_of_orientation = Vector::createFromAngle(robot_orientation);

    // Don't chip towards the friendly side
    bool is_facing_away = Angle::zero().minDiff(robot_orientation).toDegrees() < 90.0;

    bool is_chip_out_of_bounds = contains(event.common.world.field().fieldBoundary(),
        event.common.robot.position() + direction_of_orientation.normalize(YEET_CHIP_DISTANCE_METERS));

    return is_facing_away && !is_chip_out_of_bounds;
}