#pragma once

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/passing/field_pitch_division.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/geom/point.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

template <class ZoneEnum>
using ZonePassMap = std::unordered_map<ZoneEnum, PassWithRating>;

template <class ZoneEnum>
class PassEvaluation
{
    static_assert(std::is_enum<ZoneEnum>::value,
                  "PassEvaluation: ZoneEnum must be an zone id enum");

   public:
    /**
     * Create a new PassEvaluation with the best pass in each zone
     *
     * @param pitch_division The FieldPitchDivision that was used to create
     *                       this pass evaluation.
     * @param best_pass_in_zone A map of the best passes in each zone
     * @param passing_config The passing_config
     * @param timestamp The timestamp this pass evaluation was created
     */
    explicit PassEvaluation(
        std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
        ZonePassMap<ZoneEnum> best_pass_in_zones,
        std::shared_ptr<const PassingConfig> passing_config, Timestamp timestamp);

    PassEvaluation() = delete;

    /**
     * Get the best pass on the entire field.
     *
     * @returns PassWithRating containing the best pass
     */
    PassWithRating getBestPassOnField() const;

    /**
     * Given a set of zone_ids, returns the best PassWithRating in those zones
     *
     * @throws std::invalid_argument if the zone_ids set is empty or if the zone_ids
     *         are out of bounds
     * @param zone_ids A set of zone_ids to find the best pass in
     * @return PassWithRating w/ the best pass in the given zones
     */
    PassWithRating getBestPassInZones(const std::unordered_set<ZoneEnum>& zone_ids) const;

    /**
     * Returns the field pitch division this pass evaluation was computed for
     *
     * @return FieldPitchDivision defining how the field is divided
     */
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> getFieldPitchDivsion() const;

    /**
     * Returns the best zones to send robots to receive a pass in
     *
     * @param world The world to rank the zones for
     * @param position The location from where the pass will be taken
     * @return vector of sorted ZoneEnums, with the highest quality zones first
     */
    std::vector<ZoneEnum> rankZonesForReceiving(const World& world,
                                                const Point& pass_position) const;

    /**
     * Returns a timestamp of when this pass evaluation was created
     *
     * @return Timestamp the timestamp of when this pass evaluation was created
     */
    Timestamp getEvaluationTime() const;

   private:
    // The pitch division this pass evaluation was computed for
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division_;

    // Stores the best passes
    ZonePassMap<ZoneEnum> best_pass_in_zones_;

    // Stores the passing config
    std::shared_ptr<const PassingConfig> passing_config_;

    // The timestamp when this evaluation was created
    Timestamp timestamp_;
};

#include "software/ai/passing/pass_evaluation.hpp"
