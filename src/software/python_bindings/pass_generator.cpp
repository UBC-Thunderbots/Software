#include "software/ai/passing/pass_generator.hpp"

#include <pybind11/pybind11.h>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/geom/rectangle.h"
#include "software/python_bindings/pass_utilities.h"
#include "software/python_bindings/python_binding_utilities.h"

namespace py = pybind11;

using EighteenZonePassGenerator  = PassGenerator<EighteenZoneId>;
using EighteenZonePassEvaluation = PassEvaluation<EighteenZoneId>;

/**
 * Creates a PassGenerator<EighteenZoneId> with an EighteenZonePitchDivision created
 * from the given world's field and a dict with the parameter overrides for
 * PassingConfig.
 *
 * @param world the world
 * @param passing_config_dict dict of overridden parameters for the PassingConfig.
 * @return an EighteenZonePassGenerator
 */
EighteenZonePassGenerator createEighteenZonePassGenerator(const World& world,
                                                          py::dict passing_config_dict)
{
    auto passing_config = std::make_shared<PassingConfig>();
    updateDynamicParametersConfigFromDict(passing_config, passing_config_dict);

    auto pitch_division = std::make_shared<EighteenZonePitchDivision>(world.field());

    return EighteenZonePassGenerator(pitch_division, passing_config);
}

/**
 * Gets the best pass for each of the 18 zones and returns them in a list of dicts
 * as per pass_utilities.cpp.
 * @tparam ZoneEnum a ZoneEnum type corresponding to a pitch division
 * @param generator a PassGenerator
 * @param world the world
 * @return a list of dicts describing the best pass in each of the zones.
 */
template <typename ZoneEnum>
py::list getBestPassesForAllZones(PassGenerator<ZoneEnum>& generator, const World& world)
{
    py::list result;
    auto evaluation = generator.generatePassEvaluation(world);

    for (auto zone_id : evaluation.getFieldPitchDivsion()->getAllZoneIds())
    {
        auto pass_with_rating = evaluation.getBestPassInZones({zone_id});
        py::dict pass_with_rating_dict;
        pass_with_rating_dict["pass"]   = convertPassToDict(pass_with_rating.pass);
        pass_with_rating_dict["rating"] = pass_with_rating.rating;
        result.append(pass_with_rating_dict);
    }
    return result;
}

/**
 * Returns dicts describing the field pitch division zones for the given world.
 * @param world the world.
 * @return a list of dicts describing the rectangular zones.
 */
py::list getAllZones(const World& world)
{
    EighteenZonePitchDivision pitch_division(world.field());
    py::list result;
    for (auto zone_id : pitch_division.getAllZoneIds())
    {
        result.append(pitch_division.getZone(zone_id));
    }
    return result;
}

PYBIND11_MODULE(pass_generator, m)
{
    py::class_<Rectangle>(m, "Rectangle")
        .def(py::init<const Point&, const Point&>())
        .def("centre", &Rectangle::centre)
        .def("xLength", &Rectangle::xLength)
        .def("yLength", &Rectangle::yLength);

    py::class_<EighteenZonePassGenerator>(m, "EighteenZonePassGenerator")
        .def(py::init(&createEighteenZonePassGenerator))
        .def("getBestPassesForAllZones", &getBestPassesForAllZones<EighteenZoneId>);
    m.def("getAllZones", &getAllZones);
}
