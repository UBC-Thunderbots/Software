#include "ai/hl/stp/tactic/movespin_tactic.h"

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
    do
    {
        yield(movespin_action.updateStateAndGetNextIntent(*robot, Point(-3, 1),
                                                          Angle::ofRadians(0.5), 2));
        std::cout << "1\t";
    } while (!((robot->position() - Point(-3, 1)).len() < 0.1));

    do
    {
        yield(movespin_action_2.updateStateAndGetNextIntent(*robot, Point(-1, -1),
                                                            Angle::ofRadians(1), 0));
        std::cout << "2\t\t";
    } while (!((robot->position() - Point(-1, -1)).len() < 0.1));

    std::this_thread::sleep_for(std::chrono::seconds(3));
}
