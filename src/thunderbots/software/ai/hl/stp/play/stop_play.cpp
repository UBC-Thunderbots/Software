#include "ai/hl/stp/play/stop_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/goalie_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "shared/constants.h"

const std::string StopPlay::name = "Stop Play";

std::string StopPlay::getName() const
{
    return StopPlay::name;
}

bool StopPlay::isApplicable(const World &world) const
{
    return world.gameState().isStopped();
}

bool StopPlay::invariantHolds(const World &world) const
{
    return world.gameState().isStopped();
}

void StopPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // Robot assignments for the Stop Play
    //  - 1 robot will be the goalie
    //  - 2 robots will assist the goalie in blocking the ball, they will snap
    //      to the best fit semicircle around the defense area
    //  - 3 robots will stay within 0.5m of the ball, evenly spaced
    //
    //  If x represents the ball and G represents the goalie, the following
    //  diagram depicts a possible outcome of this play
    //
    // 		+--------------------+--------------------+
    // 		|           4        |                    |
    // 		|           x        |                    |
    // 		| 0       2  3       |                    |
    // 		+--+ 1               |                 +--+
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

    // We want to find the radius of the semicircle in which the defense area can be
    // inscribed, the full derivation can be found in the link below
    //
    // http://www.stumblingrobot.com/2015/10/06/
    // find-the-largest-rectangle-that-can-be-inscribed-in-a-semicircle/
    float radius = sqrt(2) * world.field().friendlyDefenseArea().width();

    do
    {
        // goalie tactic
        auto enemy_threats = Evaluation::getAllEnemyThreats(
            world.field(), world.friendlyTeam(), world.enemyTeam(), world.ball(), false);

        goalie_tactic->updateParams(
            world.ball(), world.field(), world.friendlyTeam(), world.enemyTeam(),
            enemy_threats.empty() ? std::nullopt
                                  : std::make_optional(enemy_threats.at(0)));
        std::vector<std::shared_ptr<Tactic>> result = {goalie_tactic};

        // a unit vector from the center of the goal to the ball
        Vector goal_to_ball_unit_vector =
            (world.field().friendlyGoal() - world.ball().position()).norm();

        // defense_point_center is a point on the semicircle around the friendly defense
        // area, that can block the direct path from the ball to the net.
        // robot_positioning_unit_vector will be used to position the robots on the
        // tangent line on that semicircle
        Vector robot_positiioning_unit_vector = goal_to_ball_unit_vector.perp();
        Point defense_point_center =
            world.field().friendlyGoal() - radius * goal_to_ball_unit_vector;

        // position robots on either side of the "defense point"
        Point defense_point_left = defense_point_center + robot_positiioning_unit_vector *
                                                              2 * ROBOT_MAX_RADIUS_METERS;
        Point defense_point_right =
            defense_point_center -
            robot_positiioning_unit_vector * 2 * ROBOT_MAX_RADIUS_METERS;

        move_tactics.at(0)->updateParams(
            defense_point_left,
            (world.ball().position() - defense_point_left).orientation(), 0);
        move_tactics.at(1)->updateParams(
            defense_point_right,
            (world.ball().position() - defense_point_right).orientation(), 0);

        // move the robots in a triangle around the ball, facing the ball, this behaviour
        // is very similar to the example play when there are only 3 robots, except spaced
        // 0.5m away from the ball
        Angle angle_between_robots = Angle::full() / 3;
        move_tactics.at(2)->updateParams(
            world.ball().position() +
                0.5 * Point::createFromAngle(angle_between_robots * 1),
            (angle_between_robots * 1) + Angle::half(), 0);
        move_tactics.at(3)->updateParams(
            world.ball().position() +
                0.5 * Point::createFromAngle(angle_between_robots * 2),
            (angle_between_robots * 2) + Angle::half(), 0);
        move_tactics.at(4)->updateParams(
            world.ball().position() +
                0.5 * Point::createFromAngle(angle_between_robots * 3),
            (angle_between_robots * 3) + Angle::half(), 0);

        // insert all the move tactics to the result
        result.insert(result.end(), move_tactics.begin(), move_tactics.end());
        yield(result);
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<StopPlay> factory;
