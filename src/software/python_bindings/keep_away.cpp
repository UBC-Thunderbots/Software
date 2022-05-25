#include "software/ai/evaluation/keep_away.h"

#include <pybind11/pybind11.h>

#include "software/math/math_functions.h"
#include "software/python_bindings/pass_utilities.h"
#include "software/python_bindings/python_binding_utilities.h"

namespace py = pybind11;

PYBIND11_MODULE(keep_away, m)
{
    m.def("findKeepAwayTargetPosition",
          [](const World& world, py::dict best_pass_so_far_dict) {
              Pass best_pass_so_far = createPassFromDict(best_pass_so_far_dict);
              return findKeepAwayTargetPoint(world, best_pass_so_far);
          });
    m.def("ratePasserPointForKeepAway", [](const World& world, py::dict pass_dict) {
        Pass pass = createPassFromDict(pass_dict);
        return ratePasserPointForKeepAway(pass, world.enemyTeam()) *
               circleSigmoid(Circle(world.ball().position(), 0.75), pass.passerPoint(),
                             0.05) *
               rectangleSigmoid(world.field().fieldLines(), pass.passerPoint(), 0.1);
    });
}
