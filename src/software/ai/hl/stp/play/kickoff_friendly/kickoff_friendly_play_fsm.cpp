#include "software/ai/hl/stp/play/kickoff_friendly/kickoff_friendly_play_fsm.h"

KickoffFriendlyPlayFSM::KickoffFriendlyPlayFSM(const TbotsProto::AiConfig &ai_config)
        : ai_config(ai_config),
          kickoff_chip_tactic(std::make_shared<KickoffChipTactic>()),
          shoot_tactic(std::make_shared<KickTactic>()),
          move_tactics({
                               std::make_shared<PrepareKickoffMoveTactic>(), // for robot 1
                               std::make_shared<MoveTactic>(),               // for robot 2
                               std::make_shared<MoveTactic>(),               // for robot 3
                               std::make_shared<MoveTactic>(),               // for robot 4
                               std::make_shared<MoveTactic>()                // for robot 5
                       })
{

}

void KickoffFriendlyPlayFSM::createKickoffSetupPositions(const WorldPtr &world_ptr)
{
    // Since we only have 6 robots at the maximum, the number one priority
    // is the robot doing the kickoff up front. The goalie is the second most
    // important, followed by 3 and 4 setup for offense. 5 and 6 will stay
    // back near the goalie just in case the ball quickly returns to the friendly
    // side of the field.
    //
    // 		+--------------------+--------------------+
    // 		|                    |                    |
    // 		|               3    |                    |
    // 		|                    |                    |
    // 		+--+ 5               |                 +--+
    // 		|  |                 |                 |  |
    // 		|  |               +-+-+               |  |
    // 		|2 |               |1  |               |  |
    // 		|  |               +-+-+               |  |
    // 		|  |                 |                 |  |
    // 		+--+ 6               |                 +--+
    // 		|                    |                    |
    // 		|               4    |                    |
    // 		|                    |                    |
    // 		+--------------------+--------------------+
    //

    if (kickoff_setup_positions.empty())
    {
        kickoff_setup_positions = {
                // Robot 1
                Point(world_ptr->field().centerPoint() +
                      Vector(-world_ptr->field().centerCircleRadius(), 0)),
                // Robot 2
                // Goalie positions will be handled by the goalie tactic
                // Robot 3
                Point(
                        world_ptr->field().centerPoint() +
                        Vector(-world_ptr->field().centerCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                               -1.0 / 3.0 * world_ptr->field().yLength())),
                // Robot 4
                Point(
                        world_ptr->field().centerPoint() +
                        Vector(-world_ptr->field().centerCircleRadius() - 4 * ROBOT_MAX_RADIUS_METERS,
                               1.0 / 3.0 * world_ptr->field().yLength())),
                // Robot 5
                Point(world_ptr->field().friendlyGoalpostPos().x() +
                      world_ptr->field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                      world_ptr->field().friendlyGoalpostPos().y()),
                // Robot 6
                Point(world_ptr->field().friendlyGoalpostNeg().x() +
                      world_ptr->field().defenseAreaXLength() + 2 * ROBOT_MAX_RADIUS_METERS,
                      world_ptr->field().friendlyGoalpostNeg().y()),
        };
    }
}


void KickoffFriendlyPlayFSM::setupKickoff(const Update &event)
{
    createKickoffSetupPositions(event.common.world_ptr);

    PriorityTacticVector tactics_to_run = {{}};

    // first priority requires the ability to kick and chip.
    move_tactics.at(0)->mutableRobotCapabilityRequirements() = {
            RobotCapability::Kick, RobotCapability::Chip};

    // set each tactic to its movement location.
    for (unsigned i = 0; i < kickoff_setup_positions.size(); i++)
    {
        move_tactics.at(i)->updateControlParams(kickoff_setup_positions.at(i),
                                                Angle::zero());
        tactics_to_run[0].emplace_back(move_tactics.at(i));
    }

    event.common.set_tactics(tactics_to_run);
}

// taken from free kick fsm.
void KickoffFriendlyPlayFSM::shootBall(const Update &event)
{
    WorldPtr world_ptr = event.common.world_ptr;
    PriorityTacticVector tactics_to_run = {{}};

    Point ball_pos = world_ptr->ball().position();

    shoot_tactic->updateControlParams(
            ball_pos, (shot->getPointToShootAt() - ball_pos).orientation(),
            BALL_MAX_SPEED_METERS_PER_SECOND);
    tactics_to_run[0].emplace_back(shoot_tactic);


    event.common.set_tactics(tactics_to_run);
}

void KickoffFriendlyPlayFSM::chipBall(const Update &event)
{
    WorldPtr world_ptr = event.common.world_ptr;

    PriorityTacticVector tactics_to_run = {{}};

    // adjust with testing to give us enough space to catch the ball before it goes out of bounds
    double ballX     = world_ptr->ball().position().x();
    double fieldX    = world_ptr->field().enemyGoalCenter().x() - 2;
    double negFieldY = world_ptr->field().enemyCornerNeg().y() + 0.3;
    double posFieldY = world_ptr->field().enemyCornerPos().y() - 0.3;

    Rectangle target_area_rectangle =
            Rectangle(Point(ballX, negFieldY), Point(fieldX, posFieldY));

    // sort targets by distance to enemy goal center.
    std::vector<Circle> potential_chip_targets = findGoodChipTargets(*world_ptr, target_area_rectangle);
    std::sort(potential_chip_targets.begin(), potential_chip_targets.end(),
            [world_ptr](const Circle& first_circle, const Circle& second_circle) {

                return distance(world_ptr->field().enemyGoalCenter(), first_circle.origin()) <
                        distance(world_ptr->field().enemyGoalCenter(), second_circle.origin());
            });

    Point target = world_ptr->field().centerPoint() + Vector(world_ptr->field().xLength() / 6, 0);

    if (!potential_chip_targets.empty())
    {
        target = potential_chip_targets[0].origin();
    }

    kickoff_chip_tactic->updateControlParams(
            world_ptr->ball().position(),
            target
    );

    tactics_to_run[0].emplace_back(kickoff_chip_tactic);

    event.common.set_tactics(tactics_to_run);
}

bool KickoffFriendlyPlayFSM::isSetupDone(const Update &event)
{
    return !event.common.world_ptr->gameState().isSetupState();
}

bool KickoffFriendlyPlayFSM::isPlaying(const Update& event)
{
    return event.common.world_ptr->gameState().isPlaying();
}

bool KickoffFriendlyPlayFSM::shotFound(const Update &event)
{
    shot = calcBestShotOnGoal(event.common.world_ptr->field(),
                              event.common.world_ptr->friendlyTeam(),
                              event.common.world_ptr->enemyTeam(),
                              event.common.world_ptr->ball().position(), TeamType::ENEMY);
    return shot.has_value() &&
           shot->getOpenAngle() >
           Angle::fromDegrees(
                   ai_config.attacker_tactic_config().min_open_angle_for_shot_deg());
}