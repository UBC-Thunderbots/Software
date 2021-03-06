#pragma once

#include <cstdlib>
#include <iostream>
#include <set>
#include <vector>

#include "software/ai/passing/pass_with_rating.h"
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
     *                          TODO
     */
    explicit PassEvaluation(const FieldPitchDivision& pitch_division,
                            std::vector<PassWithRating> best_pass_in_zones, Timestamp timestamp);

    PassEvaluation() = delete;

    /**
     * Get the best pass on the entire field.
     *
     * @returns PassWithRating containing the best pass
     */
    PassWithRating getBestPassOnField() const;

    /**
     * Given the zone_id, returns the best PassWithRating
     *
     * @param zone_id
     * @return PassWithRating w/ the best pass in the given zone
     */
    PassWithRating getBestPassInZone(unsigned zone_id) const;

    /**
     * Given a set of zone_ids, returns the best PassWithRating in those zones
     *
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
    const FieldPitchDivision& getFieldPitchDivsion() const;

    /**
     * Returns a timestamp of when this pass evaluation was created
     *
     * @return Timestamp the timestamp of when this pass evaluation was created
     */
    Timestamp getEvaluationTime() const;

   private:

    Timestamp timestamp_;
    FieldPitchDivision pitch_division_;
    std::vector<PassWithRating> best_pass_in_zones_;
};
