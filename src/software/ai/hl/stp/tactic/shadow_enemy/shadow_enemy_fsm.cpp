#include "software/ai/hl/stp/tactic/shadow_enemy/shadow_enemy_fsm.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/move_primitive.h"
#include "software/geom/algorithms/distance.h"

ShadowEnemyFSM::ShadowEnemyFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
        : TacticFSM<ShadowEnemyFSM>(ai_config_ptr)
{
}

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
        bool near_ball =
            distance(event.common.world_ptr->ball().position(),
                     enemy_threat_opt.value().robot.position()) < ENEMY_NEAR_BALL_DIST_M;

        return near_ball;
    }

    return false;
}



/**
 * Here we are checking if the ray going starting from the ball going through
 * the shadow defender is intersecting with the goal if this is the case
 * we will now attempt to steal the ball as we are not jeoparadizing a shot on net
 *
 *               XX   <-- Enemy
 *               XX
 *                 O  <-- Ball
 *                  \
 *                   ++   <--- Shadow defender
 *                   ++
 *                      \
 *                       \
 *                        \
 *                       +-\------------------+
 *                       |  \                 |
 *                       |   \     ++         |
 *                       |    \    ++    <-- Goalie
 *+----------------------+----|----------|----+------------------+
 */
bool ShadowEnemyFSM::blockedShot(const Update &event)
{
    Point ball_position = event.common.world_ptr->ball().position();
    Ray shot_block_direction(ball_position,
                             event.common.robot.position() - ball_position);
    Segment goalLine(event.common.world_ptr->field().friendlyGoal().negXNegYCorner(),
                     event.common.world_ptr->field().friendlyGoal().negXPosYCorner());
    bool ball_blocked = intersects(goalLine, shot_block_direction);
    bool is_close = distance(event.common.robot.position(), ball_position) < NEAR_PRESS_M;



    return (ball_blocked & is_close);
}


bool ShadowEnemyFSM::contestedBall(const Update &event)
{
    return event.common.robot.breakbeamTripped();
}

void ShadowEnemyFSM::blockPass(const Update &event)
{
    std::optional<EnemyThreat> enemy_threat_opt = event.control_params.enemy_threat;
    auto ball_position = event.common.world_ptr->ball().position();
    auto face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();

    // If no enemy_threat is found, the robot will default to blocking
    // the possible shot on net

    Point position_to_block =
        ball_position +
        (event.common.world_ptr->field().friendlyGoalCenter() - ball_position)
            .normalize(event.control_params.shadow_distance);
    if (enemy_threat_opt.has_value())
    {
        position_to_block =
            findBlockPassPoint(ball_position, enemy_threat_opt.value().robot,
                               event.control_params.shadow_distance);
    };

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, position_to_block, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE, TbotsProto::DribblerMode::OFF,
        TbotsProto::BallCollisionType::AVOID,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0}));
}

void ShadowEnemyFSM::blockShot(const Update &event,
                               boost::sml::back::process<MoveFSM::Update> processEvent)
{
    std::optional<EnemyThreat> enemy_threat_opt = event.control_params.enemy_threat;
    Point ball_position = event.common.world_ptr->ball().position();
    Angle face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();

    // If no enemy_threat is found, the robot will default to blocking
    // the possible shot on net

    Point position_to_block =
        ball_position +
        (event.common.world_ptr->field().friendlyGoalCenter() - ball_position)
            .normalize(event.control_params.shadow_distance);
    if (enemy_threat_opt.has_value())
    {
        position_to_block = findBlockShotPoint(
            event.common.robot, event.common.world_ptr->field(),
            event.common.world_ptr->friendlyTeam(), event.common.world_ptr->enemyTeam(),
            enemy_threat_opt.value().robot, event.control_params.shadow_distance);
    };

    MoveFSM::ControlParams control_params{
        .destination             = position_to_block,
        .final_orientation       = face_ball_orientation,
        .dribbler_mode           = TbotsProto::DribblerMode::OFF,
        .ball_collision_type     = TbotsProto::BallCollisionType::AVOID,
        .auto_chip_or_kick       = AutoChipOrKick{AutoChipOrKickMode::OFF, 0},
        .max_allowed_speed_mode  = TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        .obstacle_avoidance_mode = TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE};

    processEvent(MoveFSM::Update(control_params, event.common));
}

/**
 * A shadow defender decides to steal when the enemy is facing the goal
 * within a certain angle
 *
 *                        XX   <-- Enemy that is facing the net
 *                        XX
 *                          O  <-- Ball
 *
 *                          ++   Shadow defender will attempt to take
 *                          ++   the ball since the enemy is facing
 *                                the net
 *
 *
 *                       +--------------------+
 *                       |                    |
 *                       |         ++         |
 *                       |         ++    <-- Goalie
 *+----------------------+---------++---------+------------------+
 *
 * In this example the shadower will back off since the enemy is hiding
 * the ball
 *
 *
 *                          O  <-- Ball
 *                        XX   <-- Enemy that is not facing the net
 *                        XX
 *
 *
 *                          ++   Shadow defender will back off
 *                          ++   the ball since the enemy is facing
 *                               away
 *                       +--------------------+
 *                       |                    |
 *                       |         ++         |
 *                       |         ++    <-- Goalie
 *+----------------------+---------++---------+------------------+
 */
void ShadowEnemyFSM::goAndSteal(const Update &event)
{
    Point ball_position = event.common.world_ptr->ball().position();
    Angle face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();

    std::optional<EnemyThreat> enemy_threat_opt = event.control_params.enemy_threat;
    bool go_for_ball                            = true;
    Vector goal_direction =
        event.common.world_ptr->field().friendlyGoalCenter() - ball_position;
    Vector enemy_angle = Vector();


    Point enemy_face = Point();
    if (enemy_threat_opt.has_value())
    {
        enemy_angle =
            Vector::createFromAngle(enemy_threat_opt.value().robot.orientation());
        enemy_face = enemy_threat_opt.value().robot.position() +
                     enemy_angle.normalize(DIST_TO_FRONT_OF_ROBOT_METERS);
        face_ball_orientation =
            (enemy_face - event.common.robot.position()).orientation();
        // Here we check if the enemy is facing the goal if not we just shadow
        go_for_ball =
            std::acos((enemy_angle.normalize()).dot(goal_direction.normalize())) <
            ENEMY_FACE_RADIANS;
    }

    /*
     *              Go for ball is checking if the ball is within a certain angle
     *              of the net
     *                                XX   <-- Enemy that is facing the net
     *                                XX
     *                              /  O  \<-- Ball
     *                             /        \
     *                       +----/----------\----+
     *                       |   /             \  |
     *                       |  /       ++      \ |
     *                       | /        ++    <--\Goalie
     *+----------------------+---------++---------+------------------+
     */

    if (go_for_ball)
    {
        // Here if we see have an enemy_threat and it's facing the net
        // we trust that it has the ball
        // and go for it's face
        if (enemy_threat_opt.has_value())
        {
            event.common.set_primitive(std::make_unique<MovePrimitive>(
                event.common.robot, enemy_face, face_ball_orientation,
                TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
                TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
                AutoChipOrKick{AutoChipOrKickMode::OFF, 0.0}));
        }
        else
        {
            event.common.set_primitive(std::make_unique<MovePrimitive>(
                event.common.robot, ball_position, face_ball_orientation,
                TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
                TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
                TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
                AutoChipOrKick{AutoChipOrKickMode::OFF, 0.0}));
        }
    }
    else
    {
        // If the enemy is facing away from the net we shadow them
        enemy_angle =
            Vector::createFromAngle(enemy_threat_opt.value().robot.orientation());
        Point shadow_ball_position =
            enemy_threat_opt.value().robot.position() +
            (goal_direction.normalize(event.control_params.shadow_distance));

        event.common.set_primitive(std::make_unique<MovePrimitive>(
            event.common.robot, shadow_ball_position, face_ball_orientation,
            TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
            TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
            TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
            AutoChipOrKick{AutoChipOrKickMode::OFF, 0.0}));
    }
}

void ShadowEnemyFSM::stealAndPull(const Update &event)
{
    Point ball_position = event.common.world_ptr->ball().position();
    Angle face_ball_orientation =
        (ball_position - event.common.robot.position()).orientation();

    Vector direction_to_pull =
        (event.common.robot.position() - ball_position).normalize();
    Point pull_to_here = direction_to_pull + event.common.robot.position();

    event.common.set_primitive(std::make_unique<MovePrimitive>(
        event.common.robot, pull_to_here, face_ball_orientation,
        TbotsProto::MaxAllowedSpeedMode::PHYSICAL_LIMIT,
        TbotsProto::ObstacleAvoidanceMode::AGGRESSIVE,
        TbotsProto::DribblerMode::MAX_FORCE, TbotsProto::BallCollisionType::ALLOW,
        AutoChipOrKick{AutoChipOrKickMode::OFF, 0.0}));
}
