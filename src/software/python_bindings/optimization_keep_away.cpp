#include <pybind11/pybind11.h>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass.h"
#include "software/geom/point.h"
#include "software/optimization/gradient_descent_optimizer.h"
#include "software/python_bindings/pass_utilities.h"
#include "software/python_bindings/python_binding_utilities.h"

namespace py = pybind11;

Point findKeepAwayTargetPosition(const World& world, const Point& possessor_position,
                                 const Pass& best_pass_so_far,
                                 std::shared_ptr<const PassingConfig> passing_config)
{
    // TODO: arbitrary radius
    Circle keepaway_movement_region(possessor_position, 0.3);
    const auto keepaway_point_cost = [&](const std::array<double, 2>& passer_pt_array) {
        Point passer_pt(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
        Pass pass(passer_pt, best_pass_so_far.receiverPoint(), best_pass_so_far.speed());
        return ratePassEnemyRisk(world.enemyTeam(), pass, passing_config) *
               circleSigmoid(keepaway_movement_region, passer_pt, 0.1);
    };
    GradientDescentOptimizer<2> optimizer;
    auto passer_pt_array = optimizer.maximize(
        keepaway_point_cost,
        std::array<double, 2>{possessor_position.x(), possessor_position.y()},
        passing_config->getNumberOfGradientDescentStepsPerIter()->value());
    return Point(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
}

PYBIND11_MODULE(optimization_keep_away, m)
{
    m.def("findKeepAwayTargetPosition",
          [](const World& world, const Point& possessor_position,
             py::dict best_pass_so_far_dict, py::dict passing_config_dict) {
              Pass best_pass_so_far = createPassFromDict(best_pass_so_far_dict);
              auto passing_config   = std::make_shared<PassingConfig>();
              updateDynamicParametersConfigFromDict(passing_config, passing_config_dict);
              return findKeepAwayTargetPosition(world, possessor_position,
                                                best_pass_so_far, passing_config);
          });
}
