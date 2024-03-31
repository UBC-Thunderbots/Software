#pragma once

#include "proto/message_translation/tbots_protobuf.h"
#include "proto/parameters.pb.h"
#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass.h"
#include "software/ai/passing/pass_evaluation.hpp"
#include "software/ai/passing/pass_with_rating.h"
#include "software/logger/logger.h"
#include "software/optimization/gradient_descent_optimizer.hpp"
#include "software/time/timestamp.h"
#include "software/world/world.h"


// The random seed to initialize the random number generator
static const int SEED = 1010;

/**
 * This class is responsible for generating passes for us to perform
 */
template <class ZoneEnum>
class ReceiverPositionGenerator
{
    static_assert(std::is_enum<ZoneEnum>::value,
                  "PassGenerator: ZoneEnum must be a zone id enum");

public:
    /**
     * TODO (NIMA)
     * @param pitch_division The pitch division to use when looking for passes
     */
    explicit ReceiverPositionGenerator(
            std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
            TbotsProto::PassingConfig passing_config);

    /**
     * @param world
     *
     * @return
     */
    std::vector<Point> getBestReceivingPositions(const World& world, unsigned int num_positions);


private:
    Point getBestReceivingPositionInZone(const ZoneEnum zone_id, const World& world);

    // Pitch division
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division_;

    // Passing configuration
    TbotsProto::PassingConfig passing_config_;

    std::map<ZoneEnum, Point> previous_best_receiving_positions;

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;
};

template<class ZoneEnum>
ReceiverPositionGenerator<ZoneEnum>::ReceiverPositionGenerator(
        std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
        TbotsProto::PassingConfig passing_config)
        : pitch_division_(pitch_division), passing_config_(passing_config),
          random_num_gen_(SEED)
{}

template<class ZoneEnum>
std::vector<Point>
ReceiverPositionGenerator<ZoneEnum>::getBestReceivingPositions(const World &world, unsigned int num_positions)
{
    // Generate sample passes for cost visualization
//    if (passing_config_.cost_vis_config().generate_sample_passes())
//    {
//        samplePassesForVisualization(world, passing_config_);
//    }

    auto all_zones = pitch_division_->getAllZoneIds();

    // We cache the ratings of each zone to avoid rateZone being called multiple times
    // by the std::sort comparator function. This is very important as rateZone is
    // an expensive function to call.
    std::map<ZoneEnum, double> cached_ratings;
    for (const auto& zone : all_zones)
    {
        cached_ratings[zone] =
                rateZoneSmart(world, world.enemyTeam(), pitch_division_->getZone(zone),
                         world.ball().position(), passing_config_);
        if (previous_best_receiving_positions.count(zone) > 0)
        {
            // Incentivize the zones that were previously selected
            cached_ratings[zone] += 0.1;
        }
    }

    // Sort the top num_positions zones by their rating
    std::sort(all_zones.begin(), all_zones.end(),
              [&](const ZoneEnum& z1, const ZoneEnum& z2) {
                  return cached_ratings[z1] > cached_ratings[z2];
              });

    std::map<std::string, TbotsProto::Shape> zone_shapes;
    for (unsigned int i = 0; i < num_positions; i++)
    {
        zone_shapes.insert({std::to_string(i + 1),
                            *createShapeProto(pitch_division_->getZone(all_zones[i]))});
    }
    LOG(VISUALIZE) << *createDebugShapesMap(zone_shapes);

    std::vector<Point> best_positions;
    for (unsigned int i = 0; i < num_positions; ++i)
    {
        best_positions.push_back(getBestReceivingPositionInZone(all_zones[i], world));
        // TODO (NIMA): Add logic to spread out the robots. Might have to not use partial sort
    }

    // Clear the previous best receiving positions and update them with the new best positions
    previous_best_receiving_positions.clear();
    for (unsigned int i = 0; i < num_positions; ++i)
    {
        previous_best_receiving_positions[all_zones[i]] = best_positions[i];
    }

    return best_positions;
}

template<class ZoneEnum>
Point
ReceiverPositionGenerator<ZoneEnum>::getBestReceivingPositionInZone(const ZoneEnum zone_id, const World &world)
{
    Point best_receiving_position;
    double best_receiving_position_rating = 0;

    const auto previous_receiving_position_it = previous_best_receiving_positions.find(zone_id);
    if (previous_receiving_position_it != previous_best_receiving_positions.end())
    {
        best_receiving_position = previous_receiving_position_it->second;
        best_receiving_position_rating = ratePassForReceiving(world,
                                                              Pass(world.ball().position(), best_receiving_position, 5.0),
                                                              passing_config_) + 0.05;
    }

    for (int i = 0; i < 30; ++i)
    { // TODO (NIMA): Make num samples a parameter
        auto zone = pitch_division_->getZone(zone_id);

        std::uniform_real_distribution x_distribution(zone.xMin(), zone.xMax());
        std::uniform_real_distribution y_distribution(zone.yMin(), zone.yMax());

        auto pass =
                Pass(world.ball().position(),
                     Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
                     5.0); // TODO (NIMA): Pass speed should be dynamic!!

        double rating = ratePassForReceiving(world, pass, passing_config_);
        if (rating > best_receiving_position_rating)
        {
            best_receiving_position = pass.receiverPoint();
            best_receiving_position_rating = rating;
        }
    }

    return best_receiving_position;
}
