#include "software/ai/passing/pass_evaluation.h"

#include <cstdlib>
#include <iostream>
#include <unordered_set>
#include <vector>

#include "software/ai/passing/field_pitch_division.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/geom/point.h"
#include "software/time/timestamp.h"


PassEvaluation::PassEvaluation(std::shared_ptr<const FieldPitchDivision> pitch_division,
                               std::vector<PassWithRating> best_pass_in_zones,
                               Timestamp timestamp)
    : pitch_division_(pitch_division),
      best_pass_in_zones_(best_pass_in_zones),
      timestamp_(timestamp)
{
    if (pitch_division_->getTotalNumberOfZones() != best_pass_in_zones.size())
    {
        throw std::invalid_argument(
            "pass evaluation does not corresponding to pitch division");
    }
}

PassWithRating PassEvaluation::getBestPassOnField() const
{
    auto best_pass =
        std::max_element(best_pass_in_zones_.begin(), best_pass_in_zones_.end(),
                         [](const PassWithRating& pass_a, const PassWithRating& pass_b) {
                             return pass_a.rating < pass_b.rating;
                         });
    return *best_pass;
}

PassWithRating PassEvaluation::getBestPassInZones(
    const std::unordered_set<unsigned>& zone_ids) const
{
    if (zone_ids.size() == 0)
    {
        throw std::invalid_argument("no zone_ids provided");
    }

    std::for_each(zone_ids.begin(), zone_ids.end(), [this](unsigned zone) {
        if (zone < 1 || zone >= pitch_division_->getTotalNumberOfZones())
        {
            throw std::invalid_argument("zone_id is out of bounds");
        }
    });

    auto best_pass = std::max_element(
        zone_ids.begin(), zone_ids.end(), [this](unsigned zone_a, unsigned zone_b) {
           return best_pass_in_zones_[zone_a - 1].rating <
                   best_pass_in_zones_[zone_b - 1].rating;
        });

    return best_pass_in_zones_[*best_pass - 1];
}

std::shared_ptr<const FieldPitchDivision> PassEvaluation::getFieldPitchDivsion() const
{
    return pitch_division_;
}

Timestamp PassEvaluation::getEvaluationTime() const
{
    return timestamp_;
}
