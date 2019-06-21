#include "ai/hl/stp/tactic/patrol_tactic.h"

#include "ai/hl/stp/action/move_action.h"
#include "ai/hl/stp/action/stop_action.h"
#include "util/logger/init.h"

PatrolTactic::PatrolTactic(const std::vector<Point> &points, double dist_from_points)
    : Tactic(true),
      patrol_points(points),
      dist_from_points(dist_from_points),
      orientation(Angle::zero()),
      final_speed(0),
      patrol_point_index(0)
{
}

std::string PatrolTactic::getName() const
{
    return "Patrol Tactic";
}

void PatrolTactic::updateParams(Angle orientation, double final_speed)
{
    // Update the parameters stored by this Tactic
    this->orientation = orientation;
    this->final_speed = final_speed;
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

    MoveAction move_action = MoveAction(this->dist_from_points);
    do
    {
        auto next_intent = move_action.updateStateAndGetNextIntent(
            *robot, patrol_points.at(patrol_point_index), orientation, final_speed);
        if (!next_intent || move_action.done())
        {
            patrol_point_index = (patrol_point_index + 1) % patrol_points.size();
            next_intent        = move_action.updateStateAndGetNextIntent(
                *robot, patrol_points.at(patrol_point_index), orientation, final_speed);
        }

        yield(std::move(next_intent));
    } while (true);
}
