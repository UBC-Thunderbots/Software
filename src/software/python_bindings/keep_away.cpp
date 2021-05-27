#include "software/ai/evaluation/keep_away.h"

#include <pybind11/pybind11.h>

#include "software/python_bindings/pass_utilities.h"
#include "software/python_bindings/python_binding_utilities.h"

namespace py = pybind11;

PYBIND11_MODULE(keep_away, m)
{
    m.def("findKeepAwayTargetPosition",
          [](const World& world, const Point& possessor_position,
             py::dict best_pass_so_far_dict, py::dict passing_config_dict) {
              Pass best_pass_so_far = createPassFromDict(best_pass_so_far_dict);
              auto passing_config   = std::make_shared<PassingConfig>();
              updateDynamicParametersConfigFromDict(passing_config, passing_config_dict);
              return findKeepAwayTargetPoint(possessor_position, best_pass_so_far, world,
                                             passing_config);
          });
}
