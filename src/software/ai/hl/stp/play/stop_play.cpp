#include "software/ai/hl/stp/play/stop_play.h"

#include "shared/constants.h"
#include "software/ai/hl/stp/tactic/goalie_tactic.h"
#include "software/ai/hl/stp/tactic/move_tactic.h"
#include "software/util/design_patterns/generic_factory.h"

StopPlay::StopPlay(std::shared_ptr<const PlayConfig> config)
{
    play_config = config;
}

bool StopPlay::isApplicable(const World &world) const
{
    return world.gameState().isStopped();
}

bool StopPlay::invariantHolds(const World &world) const
{
    return world.gameState().isStopped();
}

void StopPlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world)
{
    // Robot assignments for the Stop Play
    //  - 1 robot will be the goalie
    //  - 2 robots will assist the goalie in blocking the ball, they will snap
    //      to the best fit semicircle around the defense area
    //  - 3 robots will stay within 0.5m of the ball, evenly spaced, also blocking the
    //  goal
    //
    //  If x represents the ball and G represents the goalie, the following, also blocking
    //  the goal diagram depicts a possible outcome of this play
    //
    // 		+--------------------+--------------------+
    // 		|                    |                    |
    // 		|         4  x       |                    |
    // 		| 0       2          |                    |
    // 		+--+ 1     3         |                 +--+
    // 		|  |                 |                 |  |
    // 		|G |               +-+-+               |  |
    // 		|  |               |   |               |  |
    // 		|  |               +-+-+               |  |
    // 		|  |                 |                 |  |
    // 		+--+                 |                 +--+
    // 		|                    |                    |
    // 		|                    |                    |
    // 		|                    |                    |
    // 		+--------------------+--------------------+


    std::vector<std::shared_ptr<MoveTactic>> move_tactics = {
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true), std::make_shared<MoveTactic>(true),
        std::make_shared<MoveTactic>(true)};

    auto goalie_tactic = std::make_shared<GoalieTactic>(
        world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam());

    // we want to find the radius of the semicircle in which the defense area can be
    // inscribed, this is so the robots can snap to that semicircle and not enter the
    // defense area. The full derivation can be found in the link below
    //
    // http://www.stumblingrobot.com/2015/10/06/
    // find-the-largest-rectangle-that-can-be-inscribed-in-a-semicircle/
    float semicircle_radius =
        sqrtf(2) * static_cast<float>(world.field().friendlyDefenseArea().yLength());

    do
    {
        // goalie tactic
        auto enemy_threats = getAllEnemyThreats(world.field(), world.friendlyTeam(),
                                                world.enemyTeam(), world.ball(), false);

        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic};

        // a unit vector from the center of the goal to the ball, this vector will be used
        // for positioning all the robots (excluding the goalie). The positioning vector
        // will be used to position robots tangent to the goal_to_ball_unit_vector
        Vector goal_to_ball_unit_vector =
            (world.field().friendlyGoalCenter() - world.ball().position()).normalize();
        Vector robot_positioning_unit_vector = goal_to_ball_unit_vector.perpendicular();

        // goal_defense_point_center is a point on the semicircle around the friendly
        // defense area, that can block the direct path from the ball to the net.
        Point goal_defense_point_center = world.field().friendlyGoalCenter() -
                                          semicircle_radius * goal_to_ball_unit_vector;

        // position robots on either side of the "goal defense point"
        Point goal_defense_point_left =
            goal_defense_point_center +
            robot_positioning_unit_vector * 2 * ROBOT_MAX_RADIUS_METERS;
        Point goal_defense_point_right =
            goal_defense_point_center -
            robot_positioning_unit_vector * 2 * ROBOT_MAX_RADIUS_METERS;

        move_tactics.at(0)->updateControlParams(
            goal_defense_point_left,
            (world.ball().position() - goal_defense_point_left).orientation(), 0);
        move_tactics.at(1)->updateControlParams(
            goal_defense_point_right,
            (world.ball().position() - goal_defense_point_right).orientation(), 0);

        // ball_defense_point_center is a point on the circle around the ball that the
        // line from the center of the goal to the ball intersects. A robot will be placed
        // on that line, and the other two will be on either side
        // We add an extra robot radius as a buffer to be extra safe we don't break any
        // rules by getting too close
        Point ball_defense_point_center =
            world.ball().position() +
            (0.5 + 2 * ROBOT_MAX_RADIUS_METERS) * goal_to_ball_unit_vector;
        Point ball_defense_point_left =
            ball_defense_point_center -
            robot_positioning_unit_vector * 4 * ROBOT_MAX_RADIUS_METERS;
        Point ball_defense_point_right =
            ball_defense_point_center +
            robot_positioning_unit_vector * 4 * ROBOT_MAX_RADIUS_METERS;

        move_tactics.at(2)->updateControlParams(
            ball_defense_point_center,
            (world.ball().position() - ball_defense_point_center).orientation(), 0);
        move_tactics.at(3)->updateControlParams(
            ball_defense_point_left,
            (world.ball().position() - ball_defense_point_left).orientation(), 0);
        move_tactics.at(4)->updateControlParams(
            ball_defense_point_right,
            (world.ball().position() - ball_defense_point_right).orientation(), 0);

        // insert all the move tactics to the result
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield(result);
    } while (true);
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, StopPlay, PlayConfig> factory;
