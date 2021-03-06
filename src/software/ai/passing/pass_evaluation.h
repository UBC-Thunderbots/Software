#pragma once

#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <vector>
#include <memory>

#include "software/ai/passing/pass_with_rating.h"
#include "software/ai/passing/field_pitch_division.h"
#include "software/geom/point.h"
#include "software/time/timestamp.h"

class PassEvaluation
{
   public:
    /**
     * Create a new PassEvaluation with the best pass in each zone
     *
     * @param pitch_division The FieldPitchDivision that was used to create
     *                       this pass evaluation.
     * @param best_pass_in_zone A vector of the best passes in each zone,
     *                          the index in the vector should correspond
     *                          with the FieldZone enum.
     * @param timestamp The timestamp this pass evaluation was created
     */
    explicit PassEvaluation(std::shared_ptr<const FieldPitchDivision> pitch_division,
                               std::vector<PassWithRating> best_pass_in_zones,
                               Timestamp timestamp);

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
     * @raises std::invalid_argument if the zone_ids set is empty or if the zone_ids
     *         are out of bounds
     * @param zone_ids A set of zone_ids to find the best pass in
     * @return PassWithRating w/ the best pass in the given zones
     */
    PassWithRating getBestPassInZones(
        const std::unordered_set<unsigned>& zone_ids) const;

    /**
     * Returns the field pitch division this pass evaluation was computed for
     *
     * @return FieldPitchDivision defining how the field is divided
     */
    std::shared_ptr<const FieldPitchDivision> getFieldPitchDivsion() const;

    /**
     * Returns a timestamp of when this pass evaluation was created
     *
     * @return Timestamp the timestamp of when this pass evaluation was created
     */
    Timestamp getEvaluationTime() const;

   private:

    // The pitch division this pass evaluation was computed for
    std::shared_ptr<const FieldPitchDivision> pitch_division_;

    // Stores the best passes indexed by zone - 1
    std::vector<PassWithRating> best_pass_in_zones_;

    // The timestamp when this evaluation was created
    Timestamp timestamp_;
};
