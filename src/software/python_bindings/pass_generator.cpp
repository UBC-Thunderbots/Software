#include <pybind11/pybind11.h>

#include "software/ai/passing/pass_generator.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/python_bindings/python_binding_utilities.h"
#include "shared/parameter/cpp_dynamic_parameters.h"

namespace py = pybind11;

using EighteenZonePassGenerator = PassGenerator<EighteenZoneId>;

EighteenZonePassGenerator createEighteenZonePassGenerator(
    const World& world, 
    py::dict passing_config_dict)
{
    auto passing_config = std::make_shared<PassingConfig>();
    updateDynamicParametersConfigFromDict(passing_config, passing_config_dict);
    
    auto pitch_division = std::make_shared<EighteenZonePitchDivision>(world.field());

    return EighteenZonePassGenerator(pitch_division, passing_config);
}

PYBIND11_MODULE(pass_generator, m)
{
    py::class_<EighteenZonePassGenerator>(m, "EighteenZonePassGenerator")
        .def(py::init(&createEighteenZonePassGenerator));
}