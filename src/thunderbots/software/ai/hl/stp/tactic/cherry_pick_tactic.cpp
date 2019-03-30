#include "ai/hl/stp/tactic/cherry_pick_tactic.h"

#include "geom/util.h"

CherryPickTactic::CherryPickTactic(const Rectangle &target_region, bool loop_forever) :
pass_generator(0.0), target_region(target_region), Tactic(loop_forever)
{
    pass_generator.setTargetRegion(target_region);
}

std::string CherryPickTactic::getName() const {
    return "Cherry Pick Tactic";
}

void CherryPickTactic::updateParams(const World& world)
{
    this->world = world;
}

double CherryPickTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the target region
    return dist(robot.position(), target_region);
}
