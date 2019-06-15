#include "ai/hl/stp/tactic/movespin_tactic.h"
#include "util/logger/init.h"

#include <algorithm>
#include <chrono>
#include <iostream>  // std::cout, std::endl
#include <thread>    // std::this_thread::sleep_for

MoveSpinTactic::MoveSpinTactic(bool loop_forever) : Tactic(loop_forever) {}

std::string MoveSpinTactic::getName() const
{
    return "MoveSpin Tactic";
}

void MoveSpinTactic::updateParams(Point destination, AngularVelocity angular_velocity,
                                  double final_speed)
{
    // Update the parameters stored by this Tactic
    this->destination      = destination;
    this->angular_velocity = angular_velocity;
    this->final_speed      = final_speed;
}

double MoveSpinTactic::calculateRobotCost(const Robot &robot, const World &world)
{
    // Prefer robots closer to the destination
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost = (robot.position() - destination).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

void MoveSpinTactic::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    MoveSpinAction movespin_action   = MoveSpinAction();
    MoveSpinAction movespin_action_2 = MoveSpinAction();

    Point point1(-2, 0);
    Point point2(-1, -1);
    do
    {
        yield(movespin_action.updateStateAndGetNextIntent(*robot, point1,
                                                          Angle::ofRadians(3.5), 2));
        LOG(DEBUG) << "Spin tactic 1" << std::endl;
    } while (!((robot->position() - point1).len() < 0.1));

    do
    {
        yield(movespin_action_2.updateStateAndGetNextIntent(*robot, point2,
                                                            Angle::ofRadians(-2), 0));
        LOG(DEBUG) << "Spin tactic 2" << std::endl;
    } while (!((robot->position() - point2).len() < 0.1));

    std::this_thread::sleep_for(std::chrono::seconds(3));
}
