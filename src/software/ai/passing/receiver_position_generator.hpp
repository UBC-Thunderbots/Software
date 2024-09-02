#pragma once

#include <random>

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"
#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/field_pitch_division.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_with_rating.h"
#include "software/logger/logger.h"
#include "software/world/world.h"

/**
 * This class is responsible for generating the best positions for our pass
 * receivers to go to
 */
template <class ZoneEnum>
class ReceiverPositionGenerator
{
    static_assert(std::is_enum<ZoneEnum>::value,
                  "PassGenerator: ZoneEnum must be a zone id enum");

   public:
    /**
     * Creates a new ReceiverPositionGenerator
     *
     * @param pitch_division The pitch division to split the receivers into
     * @param passing_config The passing configuration to use when looking for best
     * receiving positions
     */
    explicit ReceiverPositionGenerator(
        std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
        TbotsProto::PassingConfig passing_config);

    /**
     * Generates the best receiving positions for the friendly robots to go to
     *
     * @param world The world to generate the best receiving positions based on
     * @param num_positions The number of receiving positions to generate
     * @param existing_receiver_positions A set of existing receiver positions that will
     * be avoided, if possible, when generating the new receiver positions.
     * @param pass_origin_override An optional override for the position where the pass
     * will be made from. If not provided, the ball position will be used. This could be
     * helpful if you are trying to position the receivers based on the ball's future
     * position.
     * @return A vector of
     *      min(num_positions, num_friendly_robots - existing_receiver_positions.size())
     * positions that the receivers could use.
     */
    std::vector<Point> getBestReceivingPositions(
        const World &world, unsigned int num_positions,
        const std::vector<Point> &existing_receiver_positions = {},
        const std::optional<Point> &pass_origin_override      = std::nullopt);


   private:
    /**
     * Sample more points and update the best_receiving_positions map if better
     * receiving positions in each zone were found
     *
     * @param best_receiving_positions The map of the best receiving positions for each
     * zone found so far, and their ratings.
     * @param world The world to sample receiving positions in
     * @param pass_origin The origin of the pass
     * @param zones_to_sample The subset of the zones to sample receiving positions in
     * @param num_samples_per_zone The number of samples to take per zone
     */
    void updateBestReceiverPositions(
        std::map<ZoneEnum, PassWithRating> &best_receiving_positions, const World &world,
        const Point &pass_origin, const std::vector<ZoneEnum> &zones_to_sample,
        unsigned int num_samples_per_zone);

    /**
     * Helper function for getting the top num_positions zones from the current
     * best_receiving_positions. Note that this function will attempt to spread out the
     * receivers so they are not too close to each other.
     *
     * @param best_receiving_positions The current best receiving position and rating per
     * zone
     * @param num_positions The number of top zones to get
     * @param pass_origin The origin of the pass
     * @param existing_receiver_positions The existing receiver positions to avoid
     * @return A vector of the top num_positions zones
     */
    std::vector<ZoneEnum> getTopZones(
        const std::map<ZoneEnum, PassWithRating> &best_receiving_positions,
        unsigned int num_positions, const Point &pass_origin,
        const std::vector<Point> &existing_receiver_positions);

    /**
     * Helper method for visualizing the best receiving positions and zones
     * @param best_receiving_positions The best receiving positions and their ratings
     * @param top_zones The ranked top zones to visualize
     */
    void visualizeBestReceivingPositionsAndZones(
        const std::map<ZoneEnum, PassWithRating> &best_receiving_positions,
        const std::vector<ZoneEnum> &top_zones);

    // Pitch division
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division_;

    // The best receiving position for each zone from the previous iteration
    std::map<ZoneEnum, Point> prev_best_receiving_positions;

    // Passing configuration
    TbotsProto::PassingConfig passing_config_;

    // A vector of shapes that will be visualized
    std::vector<TbotsProto::DebugShapes::DebugShape> debug_shapes;

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;

    // The random seed to initialize the random number generator
    static constexpr int RNG_SEED = 1010;
};

template <class ZoneEnum>
ReceiverPositionGenerator<ZoneEnum>::ReceiverPositionGenerator(
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
    TbotsProto::PassingConfig passing_config)
    : pitch_division_(pitch_division),
      passing_config_(passing_config),
      random_num_gen_(RNG_SEED)
{
}

template <class ZoneEnum>
std::vector<Point> ReceiverPositionGenerator<ZoneEnum>::getBestReceivingPositions(
    const World &world, unsigned int num_positions,
    const std::vector<Point> &existing_receiver_positions,
    const std::optional<Point> &pass_origin_override)
{
    std::map<ZoneEnum, PassWithRating> best_receiving_positions;
    debug_shapes.clear();

    Point pass_origin           = pass_origin_override.value_or(world.ball().position());
    const auto &receiver_config = passing_config_.receiver_position_generator_config();

    // Verify that the number of receiver positions requested is valid
    if (num_positions >
        (world.friendlyTeam().numRobots() - existing_receiver_positions.size()))
    {
        LOG(WARNING) << "Not enough friendly robots to assign " << num_positions
                     << " receiver positions. Assigning "
                     << world.friendlyTeam().numRobots() -
                            existing_receiver_positions.size()
                     << " receiver positions instead";
        num_positions = static_cast<unsigned int>(world.friendlyTeam().numRobots() -
                                                  existing_receiver_positions.size());
    }

    // Add the previous best sampled receiving positions with their updated rating
    for (const auto &[zone, prev_best_receiving_position] : prev_best_receiving_positions)
    {
        Pass pass = Pass::fromDestReceiveSpeed(pass_origin, prev_best_receiving_position,
                                               passing_config_);
        // Increase the rating of the previous best receiving positions to
        // discourage changing the receiver positions too much.
        double receiver_position_rating =
            rateReceivingPosition(world, pass, passing_config_) *
            receiver_config.previous_best_receiver_position_score_multiplier();
        best_receiving_positions.insert_or_assign(
            zone, PassWithRating{pass, receiver_position_rating});
    }

    // Begin by sampling a few passes per zone to get an initial estimate of the best
    // receiving zones
    updateBestReceiverPositions(best_receiving_positions, world, pass_origin,
                                pitch_division_->getAllZoneIds(),
                                receiver_config.num_initial_samples_per_zone());

    // Get the top zones based on the initial sampling
    std::vector<ZoneEnum> top_zones =
        getTopZones(best_receiving_positions, num_positions, pass_origin,
                    existing_receiver_positions);

    // Sample more passes from only the top zones and update their ranking
    updateBestReceiverPositions(best_receiving_positions, world, pass_origin, top_zones,
                                receiver_config.num_additional_samples_per_top_zone());
    std::sort(top_zones.begin(), top_zones.end(),
              [&](const ZoneEnum &z1, const ZoneEnum &z2) {
                  return best_receiving_positions.find(z1)->second.rating >
                         best_receiving_positions.find(z2)->second.rating;
              });

    // Get the top best receiving positions and update the previous best
    std::vector<Point> best_positions;
    prev_best_receiving_positions.clear();
    for (const auto zone : top_zones)
    {
        Point best_position =
            best_receiving_positions.find(zone)->second.pass.receiverPoint();
        best_positions.push_back(best_position);
        prev_best_receiving_positions.insert_or_assign(zone, best_position);
    }

    // Visualize the receiving positions and zones
    if (receiver_config.receiver_vis_config()
            .visualize_best_receiving_positions_and_zones())
    {
        visualizeBestReceivingPositionsAndZones(best_receiving_positions, top_zones);
    }

    return best_positions;
}

template <class ZoneEnum>
void ReceiverPositionGenerator<ZoneEnum>::visualizeBestReceivingPositionsAndZones(
    const std::map<ZoneEnum, PassWithRating> &best_receiving_positions,
    const std::vector<ZoneEnum> &top_zones)
{
    for (unsigned int i = 0; i < top_zones.size(); i++)
    {
        debug_shapes.push_back(*createDebugShape(pitch_division_->getZone(top_zones[i]),
                                                 std::to_string(i + 1),
                                                 std::to_string(i + 1)));

        debug_shapes.push_back(*createDebugShape(
            Circle(
                best_receiving_positions.find(top_zones[i])->second.pass.receiverPoint(),
                0.15),
            std::to_string(i + 1) + "rpg", std::to_string(i + 1) + "rpg"));
    }

    LOG(VISUALIZE) << *createDebugShapes(debug_shapes);
}

template <class ZoneEnum>
void ReceiverPositionGenerator<ZoneEnum>::updateBestReceiverPositions(
    std::map<ZoneEnum, PassWithRating> &best_receiving_positions, const World &world,
    const Point &pass_origin, const std::vector<ZoneEnum> &zones_to_sample,
    unsigned int num_samples_per_zone)
{
    for (const auto &zone_id : zones_to_sample)
    {
        auto zone = pitch_division_->getZone(zone_id);
        std::uniform_real_distribution x_distribution(zone.xMin(), zone.xMax());
        std::uniform_real_distribution y_distribution(zone.yMin(), zone.yMax());

        PassWithRating best_pass_for_receiving{Pass(Point(0, 0), Point(0, 0), 1.0), -1.0};

        // Check if we have already sampled some passes for this zone
        const auto &best_sampled_pass_iter = best_receiving_positions.find(zone_id);
        if (best_sampled_pass_iter != best_receiving_positions.end())
        {
            best_pass_for_receiving = best_sampled_pass_iter->second;
        }

        // Randomly sample receiving positions in the zone
        for (unsigned int i = 0; i < num_samples_per_zone; ++i)
        {
            auto pass = Pass::fromDestReceiveSpeed(
                pass_origin,
                Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
                passing_config_);
            double rating = rateReceivingPosition(world, pass, passing_config_);

            if (rating > best_pass_for_receiving.rating)
            {
                best_pass_for_receiving = PassWithRating{pass, rating};
            }
        }

        best_receiving_positions.insert_or_assign(zone_id, best_pass_for_receiving);
    }
}

template <class ZoneEnum>
std::vector<ZoneEnum> ReceiverPositionGenerator<ZoneEnum>::getTopZones(
    const std::map<ZoneEnum, PassWithRating> &best_receiving_positions,
    unsigned int num_positions, const Point &pass_origin,
    const std::vector<Point> &existing_receiver_positions)
{
    std::vector<ZoneEnum> top_zones;

    // Sort the zones based on initial ratings
    auto all_zones = pitch_division_->getAllZoneIds();
    std::sort(all_zones.begin(), all_zones.end(),
              [&](const ZoneEnum &z1, const ZoneEnum &z2) {
                  return best_receiving_positions.find(z1)->second.rating >
                         best_receiving_positions.find(z2)->second.rating;
              });

    // Iterate through the zones in descending order of rating and select them as top
    // zones if they are not too close to the previous selected zones.
    const Angle min_angle_diff_between_receivers =
        Angle::fromDegrees(passing_config_.receiver_position_generator_config()
                               .min_angle_between_receivers_deg());

    for (unsigned int i = 0; i < all_zones.size() && top_zones.size() < num_positions;
         i++)
    {
        Angle curr_pass_angle =
            best_receiving_positions.find(all_zones[i])->second.pass.passerOrientation();

        // Check that none of the previously selected top zones are close to the current
        // candidate zone
        bool no_prev_receivers_close =
            std::none_of(top_zones.begin(), top_zones.end(), [&](const ZoneEnum &zone) {
                return curr_pass_angle.minDiff(best_receiving_positions.find(zone)
                                                   ->second.pass.passerOrientation()) <
                       min_angle_diff_between_receivers;
            });

        // and none of the existing receiver positions are close to the current
        // candidate zone
        no_prev_receivers_close =
            no_prev_receivers_close &&
            std::none_of(
                existing_receiver_positions.begin(), existing_receiver_positions.end(),
                [&](const Point &existing_receiver_position) {
                    return curr_pass_angle.minDiff(
                               (existing_receiver_position - pass_origin).orientation()) <
                           min_angle_diff_between_receivers;
                });

        if (no_prev_receivers_close)
        {
            top_zones.push_back(all_zones[i]);
        }
    }

    // If we did not find enough receiver positions, add the remaining top zones
    if (top_zones.size() < num_positions)
    {
        LOG(WARNING)
            << "Not enough receiver positions were found. Expected to find "
            << num_positions << " receiver positions, but only found " << top_zones.size()
            << ". Consider reducing 'min_angle_between_receivers_deg' in the dynamic parameters";
        for (unsigned int i = 0; i < all_zones.size() && top_zones.size() < num_positions;
             i++)
        {
            if (std::find(top_zones.begin(), top_zones.end(), all_zones[i]) ==
                top_zones.end())
            {
                top_zones.push_back(all_zones[i]);
            }
        }
    }

    return top_zones;
}
