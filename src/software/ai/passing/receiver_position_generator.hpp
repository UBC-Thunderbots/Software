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
template<class ZoneEnum>
class ReceiverPositionGenerator {
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
    std::vector<Point> getBestReceivingPositions(const World &world, unsigned int num_positions);


private:
    /**
     * TODO (NIMA)
     * @note Updates best_receiving_positions
     *
     * @param world
     * @param zones_to_sample
     * @param num_samples_per_zone
     */
    void
    updateBestReceiverPositions(const World &world, const std::vector<ZoneEnum> &zones_to_sample, unsigned int num_samples_per_zone);

    // Pitch division
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division_;

    // Passing configuration
    TbotsProto::PassingConfig passing_config_;

    std::map<ZoneEnum, PassWithRating> best_receiving_positions;
    std::map<ZoneEnum, Point> prev_best_receiving_positions;

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;
};

template<class ZoneEnum>
ReceiverPositionGenerator<ZoneEnum>::ReceiverPositionGenerator(
        std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
        TbotsProto::PassingConfig passing_config)
        : pitch_division_(pitch_division), passing_config_(passing_config),
          random_num_gen_(SEED) {}

template<class ZoneEnum>
std::vector<Point>
ReceiverPositionGenerator<ZoneEnum>::getBestReceivingPositions(const World &world, unsigned int num_positions)
{
    best_receiving_positions.clear();

    for (const auto& [zone, prev_best_receiving_position] : prev_best_receiving_positions) {
        // TODO (NIMA): Pass speed should be dynamic!!
        Pass pass(world.ball().position(),
                  prev_best_receiving_position,
                  5.0);
        best_receiving_positions.insert_or_assign(zone, PassWithRating{pass, rateReceivingPosition(world, pass,
                                                                                          passing_config_) * 1.1}); // TODO (NIMA): Make this a parameter
    }

    auto all_zones = pitch_division_->getAllZoneIds();

    // Begin by sampling a few passes per zone to get an initial estimate of the best
    // receiving zones
    updateBestReceiverPositions(world, all_zones, 5); // TODO (NIMA): Make this a parameter

    // Sort the zones based on initial ratings
    std::vector<ZoneEnum> top_zones(num_positions);
    std::partial_sort_copy(all_zones.begin(), all_zones.end(), top_zones.begin(), top_zones.end(),
                           [&](const ZoneEnum &z1, const ZoneEnum &z2)
                           {
                               return best_receiving_positions.find(z1)->second.rating > best_receiving_positions.find(z2)->second.rating;
                           });

    updateBestReceiverPositions(world, top_zones, 20); // TODO (NIMA): Make this a parameter

    // Get the top num_positions best receiving positions and update the previous best
    std::vector<Point> best_positions;
    prev_best_receiving_positions.clear();
    for (const auto zone: top_zones) {
        Point best_position = best_receiving_positions.find(zone)->second.pass.receiverPoint();
        best_positions.push_back(best_position);
        prev_best_receiving_positions.insert_or_assign(zone, best_position);
    }

    // Visualize the best zones
    std::map<std::string, TbotsProto::Shape> zone_shapes;
    for (const auto zone: top_zones) {
        zone_shapes.insert({toString(zone),
                            *createShapeProto(pitch_division_->getZone(zone))});
    }
    LOG(VISUALIZE) << *createDebugShapesMap(zone_shapes);

    return best_positions;
}

template<class ZoneEnum>
void ReceiverPositionGenerator<ZoneEnum>::updateBestReceiverPositions(const World &world, const std::vector<ZoneEnum> &zones_to_sample,
                                                                      unsigned int num_samples_per_zone)
{
    for (const auto &zone_id: zones_to_sample) {
        auto zone = pitch_division_->getZone(zone_id);
        std::uniform_real_distribution x_distribution(zone.xMin(), zone.xMax());
        std::uniform_real_distribution y_distribution(zone.yMin(), zone.yMax());

        PassWithRating best_pass_for_receiving{Pass(Point(0, 0), Point(0, 0), 0), -1.0};

        // Check if we have already sampled some passes for this zone
        const auto &best_sampled_pass_iter = best_receiving_positions.find(zone_id);
        if (best_sampled_pass_iter != best_receiving_positions.end()) {
            best_pass_for_receiving = best_sampled_pass_iter->second;
        }

        // Sample passes in the zone
        for (unsigned int i = 0; i < num_samples_per_zone; ++i) {
            auto pass =
                    Pass(world.ball().position(),
                         Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
                         5.0); // TODO (NIMA): Pass speed should be dynamic!!

            double rating = ratePassForReceiving(world, pass, passing_config_);
            if (rating > best_pass_for_receiving.rating) {
                best_pass_for_receiving = PassWithRating{pass, rating};
            }
        }

        best_receiving_positions.insert_or_assign(zone_id, best_pass_for_receiving);
    }
}
