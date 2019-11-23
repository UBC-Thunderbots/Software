#include "software/ai/evaluation/find_open_areas.h"

#include "software/ai/evaluation/indirect_chip.h"
#include "software/util/parameter/dynamic_parameters.h"

std::vector<Circle> Evaluation::findGoodChipTargets(const World& world)
{
    Rectangle target_area_rectangle =
        findBestChipTargetArea(world, Util::DynamicParameters->getEvaluationConfig()
                                          ->getIndirectChipConfig()
                                          ->ChipTargetAreaInset()
                                          ->value());

    std::vector<Point> enemy_locations;
    for (Robot robot : world.enemyTeam().getAllRobots())
    {
        enemy_locations.emplace_back(robot.position());
    }

    return findOpenCircles(target_area_rectangle, enemy_locations);
}
