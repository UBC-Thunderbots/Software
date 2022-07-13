#pragma once

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "proto/parameters.pb.h"
#include "software/ai/passing/field_pitch_division.h"
#include "software/ai/passing/pass.h"
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
        TbotsProto::PassingConfig passing_config, Timestamp timestamp);

    PassEvaluation() = delete;

    /**
     * Get the best pass on the entire field.
     *
     * @returns PassWithRating containing the best pass
     */
    PassWithRating getBestPassOnField() const;


    /**
     * Get the best N passes on the entire field, where each pass is from a different zone
     *
     * @returns PassWithRating containing the best pass
     */
    std::vector<PassWithRating> getBestNPassesOnField(int n) const;


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
    TbotsProto::PassingConfig passing_config_;

    // The timestamp when this evaluation was created
    Timestamp timestamp_;
};
template <class ZoneEnum>
PassEvaluation<ZoneEnum>::PassEvaluation(
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
    std::unordered_map<ZoneEnum, PassWithRating> best_pass_in_zones,
    TbotsProto::PassingConfig passing_config, Timestamp timestamp)
    : pitch_division_(pitch_division),
      best_pass_in_zones_(best_pass_in_zones),
      passing_config_(passing_config),
      timestamp_(timestamp)
{
}

template <class ZoneEnum>
PassWithRating PassEvaluation<ZoneEnum>::getBestPassOnField() const
{
    auto best_pass =
        std::max_element(best_pass_in_zones_.begin(), best_pass_in_zones_.end(),
                         [](const std::pair<ZoneEnum, PassWithRating>& p1,
                            const std::pair<ZoneEnum, PassWithRating>& p2) {
                             return p1.second.rating < p2.second.rating;
                         });
    return best_pass->second;
}

template <class ZoneEnum>
std::vector<PassWithRating> PassEvaluation<ZoneEnum>::getBestNPassesOnField(int n) const
{
    auto best_passes_copy = ZonePassMap<ZoneEnum>(best_pass_in_zones_) ;
    std::vector<PassWithRating> best_n_passes;

    for (int i = 0; i < n; i++){
        auto best_pass =
                std::max_element(best_passes_copy.begin(), best_passes_copy.end(),
                                 [](const std::pair<ZoneEnum, PassWithRating>& p1,
                                    const std::pair<ZoneEnum, PassWithRating>& p2) {
                                     return p1.second.rating < p2.second.rating;
                                 });
        best_n_passes.push_back(best_pass);
        best_passes_copy.erase(best_pass->first);
    }

    return best_n_passes;
}

template <class ZoneEnum>
PassWithRating PassEvaluation<ZoneEnum>::getBestPassInZones(
    const std::unordered_set<ZoneEnum>& zone_ids) const
{
    if (zone_ids.size() == 0)
    {
        throw std::invalid_argument("no zone_ids provided");
    }

    auto compare_pass_rating = [this](ZoneEnum zone_a, ZoneEnum zone_b) {
        return best_pass_in_zones_.at(zone_a).rating <
               best_pass_in_zones_.at(zone_b).rating;
    };

    ZoneEnum best_zone =
        *std::max_element(zone_ids.begin(), zone_ids.end(), compare_pass_rating);

    return best_pass_in_zones_.at(best_zone);
}

template <class ZoneEnum>
std::shared_ptr<const FieldPitchDivision<ZoneEnum>>
PassEvaluation<ZoneEnum>::getFieldPitchDivsion() const
{
    return pitch_division_;
}

template <class ZoneEnum>
Timestamp PassEvaluation<ZoneEnum>::getEvaluationTime() const
{
    return timestamp_;
}

template <class ZoneEnum>
std::vector<ZoneEnum> PassEvaluation<ZoneEnum>::rankZonesForReceiving(
    const World& world, const Point& pass_position) const
{
    std::vector<ZoneEnum> cherry_pick_zones = pitch_division_->getAllZoneIds();

    std::sort(cherry_pick_zones.begin(), cherry_pick_zones.end(),
              [this, &world, &pass_position](const ZoneEnum& z1, const ZoneEnum& z2) {
                  return rateZone(world.field(), world.enemyTeam(),
                                  pitch_division_->getZone(z1), pass_position,
                                  passing_config_) >
                         rateZone(world.field(), world.enemyTeam(),
                                  pitch_division_->getZone(z2), pass_position,
                                  passing_config_);
              });

//    std::sort(cherry_pick_zones.begin(), cherry_pick_zones.end(),
//              [this, &world, &pass_position](const ZoneEnum& z1, const ZoneEnum& z2) {
//                  return getBestPassInZones({z1}).rating >
//                         getBestPassInZones({z2}).rating;
//              });
    return cherry_pick_zones;
}
