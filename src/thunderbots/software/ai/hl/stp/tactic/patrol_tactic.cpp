#include "ai/hl/stp/tactic/patrol_tactic.h"

#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/action/stop_action.h"
#include "util/logger/init.h"

PatrolTactic::PatrolTactic(const std::vector<Point> &points,
                           double at_patrol_point_tolerance,
                           double linear_speed_at_patrol_points)
    : Tactic(true),
      patrol_points(points),
      at_patrol_point_tolerance(at_patrol_point_tolerance),
      orientation_at_patrol_points(Angle::zero()),
      linear_speed_at_patrol_points(linear_speed_at_patrol_points),
      patrol_point_index(0)
{
}

std::string PatrolTactic::getName() const
{
    return "Patrol Tactic";
}

void PatrolTactic::updateParams(Angle orientation_at_patrol_points)
{
    // Update the parameters stored by this Tactic
    this->orientation_at_patrol_points = orientation_at_patrol_points;
}

double PatrolTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    if (patrol_points.empty())
    {
        return 0.0;
    }
    else if (this->robot)
    {
        // If we already assigned a robot to this tactic, prefer reassigning
        // that robot
        if (this->robot.value() == robot)
        {
            return 0.0;
        }
        else
        {
            return 1.0;
        }
    }
    else
    {
        // Prefer robots that are close to the current patrol point
        double dist = (robot.position() - patrol_points.at(patrol_point_index)).len();
        double cost = dist / world.field().totalLength();
        return std::clamp<double>(cost, 0, 1);
    }
}

void PatrolTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    if (patrol_points.empty())
    {
        StopAction stop_action = StopAction(false, true);
        do
        {
            LOG(WARNING) << "Running a Patrol Tactic with no patrol points" << std::endl;
            yield(stop_action.updateStateAndGetNextIntent(*robot, false));
        } while (true);
    }

    MoveAction move_action = MoveAction(this->at_patrol_point_tolerance);
    do
    {
        auto next_intent = move_action.updateStateAndGetNextIntent(
            *robot, patrol_points.at(patrol_point_index), orientation_at_patrol_points,
            linear_speed_at_patrol_points);
        if (!next_intent || move_action.done())
        {
            patrol_point_index = (patrol_point_index + 1) % patrol_points.size();
            next_intent        = move_action.updateStateAndGetNextIntent(
                *robot, patrol_points.at(patrol_point_index),
                orientation_at_patrol_points, linear_speed_at_patrol_points);
        }

        yield(std::move(next_intent));
    } while (true);
}
