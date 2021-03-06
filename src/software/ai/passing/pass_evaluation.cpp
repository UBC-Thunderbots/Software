#include <cstdlib>
#include <iostream>
#include <vector>
#include <unordered_set>

#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/ai/passing/pass_evaluation.h"
#include "software/ai/passing/field_pitch_division.h"
#include "software/geom/point.h"
#include "software/time/timestamp.h"


PassEvaluation::PassEvaluation(const FieldPitchDivision& pitch_division,
                               std::vector<PassWithRating> best_pass_in_zones,
                               Timestamp timestamp)
    : pitch_division_(pitch_division),
      best_pass_in_zones_(best_pass_in_zones),
      timestamp_(timestamp)
{
}

PassWithRating PassEvaluation::getBestPassOnField() const
{
    auto best_pass =
        std::max_element(best_pass_in_zones_.begin(), best_pass_in_zones_.end(),
                         [](const PassWithRating& pass_a, const PassWithRating& pass_b) {
                             return pass_a.rating > pass_b.rating;
                         });
    return *best_pass;
}

PassWithRating PassEvaluation::getBestPassInZone(unsigned zone_id) const
{
    if (zone_id < 1 || zone_id >= getTotalNumberOfZones())
    {
        throw std::invalid_argument("zone_id is out of bounds");
    }
    return best_pass_in_zones_[zone_id - 1];
}

PassWithRating PassEvaluation::getBestPassInZones(
    const std::unordered_set<unsigned>& zone_ids) const
{
    auto best_pass = std::max_element(
        zone_ids.begin(), zone_ids.end(), [](unsigned zone_a, unsigned zone_b) {
            if (zone_a < 1 || zone_a >= pitch_division_.getTotalNumberOfZones() ||
                zone_b < 1 || zone_b >= pitch_division_.getTotalNumberOfZones())
            {
                throw std::invalid_argument("zone_id is out of bounds");
            }

            return best_pass_in_zones_[zone_a - 1].rating <
                   best_pass_in_zones_[zone_b - 1].rating;
        });
    return *best_pass;
}

const FieldPitchDivision& PassEvaluation::getFieldPitchDivsion() const
{
    return pitch_division_;
}

Timestamp getEvaluationTime() const
{
    return timestamp_;
}
