#pragma once

#include <iomanip>
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
    std::vector<Point> getBestReceivingPositions(const World &world,
                                                 unsigned int num_positions);


   private:
    /**
     * TODO (NIMA)
     * @note Updates best_receiving_positions
     *
     * @param world
     * @param zones_to_sample
     * @param num_samples_per_zone
     */
    void updateBestReceiverPositions(const World &world,
                                     const std::vector<ZoneEnum> &zones_to_sample,
                                     unsigned int num_samples_per_zone);

    // Pitch division
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division_;

    // Passing configuration
    TbotsProto::PassingConfig passing_config_;

    std::map<ZoneEnum, PassWithRating> best_receiving_positions;
    std::map<ZoneEnum, Point> prev_best_receiving_positions;

    std::vector<TbotsProto::DebugShapes::DebugShape> debug_shapes; // TODO (NIMA): Added for debugging

    // A random number generator for use across the class
    std::mt19937 random_num_gen_;
};

template <class ZoneEnum>
ReceiverPositionGenerator<ZoneEnum>::ReceiverPositionGenerator(
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
    TbotsProto::PassingConfig passing_config)
    : pitch_division_(pitch_division),
      passing_config_(passing_config),
      random_num_gen_(SEED)
{
}

template <class ZoneEnum>
std::vector<Point> ReceiverPositionGenerator<ZoneEnum>::getBestReceivingPositions(
    const World &world, unsigned int num_positions)
{
    best_receiving_positions.clear();
    debug_shapes.clear();

    for (const auto &[zone, prev_best_receiving_position] : prev_best_receiving_positions)
    {
        // TODO (NIMA): Pass speed should be dynamic!!
        Pass pass(world.ball().position(), prev_best_receiving_position, 5.0);
        best_receiving_positions.insert_or_assign(
            zone,
            PassWithRating{pass, rateReceivingPosition(world, pass, passing_config_) *
                                     1.5});  // TODO (NIMA): Make this a parameter
    }

    auto all_zones = pitch_division_->getAllZoneIds();

    // Begin by sampling a few passes per zone to get an initial estimate of the best
    // receiving zones
    updateBestReceiverPositions(world, all_zones,
                                5);  // TODO (NIMA): Make this a parameter

    // TODO:
    //  1. Have receivers spread out,
    //  2. tune parameters like prev pass rating,
    //  3. best receiving position is sometimes close to the receiver tactic,
    //  4. make enemy interception area wider
    //  5. RatePassShootScore should have less effect on the score, specially compared to enemy interception
    //  6. RatePassShootScore sometimes blocks large circles of the field. Should probably only draw lines

    // Sort the zones based on initial ratings
    auto zone_comparator = [&](const ZoneEnum &z1, const ZoneEnum &z2) {
        return best_receiving_positions.find(z1)->second.rating >
               best_receiving_positions.find(z2)->second.rating;
    };
    std::sort(all_zones.begin(), all_zones.end(), zone_comparator);

    std::vector<ZoneEnum> top_zones;
    top_zones.push_back(all_zones[0]);
    Angle prev_pass_angle = best_receiving_positions.find(all_zones[0])->second.pass.passerOrientation();
    for (unsigned int i = 1; i < all_zones.size() && top_zones.size() < num_positions; i++)
    {
        // Only add zones that are not too close to the previous zone
        // to encourage spreading out the receivers
        Angle curr_pass_angle = best_receiving_positions.find(all_zones[i - 1])->second.pass.passerOrientation();
        if (curr_pass_angle.minDiff(prev_pass_angle) > Angle::fromDegrees(20)) // TODO (NIMA): Make this a parameter
        {
            top_zones.push_back(all_zones[i]);
            prev_pass_angle = curr_pass_angle;
        }
    }

    // Sample more passes from the top zones and update ranking
    updateBestReceiverPositions(world, top_zones, 20);  // TODO (NIMA): Make this a parameter
    std::sort(top_zones.begin(), top_zones.end(), zone_comparator);

    // Get the top best receiving positions and update the previous best
    std::vector<Point> best_positions;
    prev_best_receiving_positions.clear();
    for (const auto zone : top_zones) // TODO (NIMA): DOnt add more than num_positions!!!!!!!!!!
    {
        Point best_position =
            best_receiving_positions.find(zone)->second.pass.receiverPoint();
        best_positions.push_back(best_position);
        prev_best_receiving_positions.insert_or_assign(zone, best_position);
    }

    CHECK(best_positions.size() == num_positions) << "Number of best positions=" << best_positions.size() << " should be equal to num_positions=" << num_positions << " top_zones.size()=" << top_zones.size();

    // Visualize the best zones
    for (unsigned int i = 0; i < top_zones.size(); i++)
    {
        debug_shapes.push_back(*createDebugShape(pitch_division_->getZone(top_zones[i]), std::to_string(i+1), std::to_string(i+1)));
        debug_shapes.push_back(*createDebugShape(Circle(best_receiving_positions.find(top_zones[i])->second.pass.receiverPoint(), 0.15), std::to_string(i+1) + "p", std::to_string(i+1) + "p"));
    }
    LOG(VISUALIZE) << *createDebugShapes(debug_shapes);

    return best_positions;
}

template <class ZoneEnum>
void ReceiverPositionGenerator<ZoneEnum>::updateBestReceiverPositions(
    const World &world, const std::vector<ZoneEnum> &zones_to_sample,
    unsigned int num_samples_per_zone)
{
    for (const auto &zone_id : zones_to_sample)
    {
        auto zone = pitch_division_->getZone(zone_id);
        std::uniform_real_distribution x_distribution(zone.xMin(), zone.xMax());
        std::uniform_real_distribution y_distribution(zone.yMin(), zone.yMax());

        PassWithRating best_pass_for_receiving{Pass(Point(0, 0), Point(0, 0), 0), -1.0};

        // Check if we have already sampled some passes for this zone
        const auto &best_sampled_pass_iter = best_receiving_positions.find(zone_id);
        if (best_sampled_pass_iter != best_receiving_positions.end())
        {
            best_pass_for_receiving = best_sampled_pass_iter->second;
        }

        // Sample passes in the zone
        for (unsigned int i = 0; i < num_samples_per_zone; ++i)
        {
            auto pass = Pass(
                world.ball().position(),
                Point(x_distribution(random_num_gen_), y_distribution(random_num_gen_)),
                5.0);  // TODO (NIMA): Pass speed should be dynamic!!

            double rating = rateReceivingPosition(world, pass, passing_config_);

//            std::stringstream stream;
//            stream << std::fixed << std::setprecision(3) << rating;
//            debug_shapes.push_back(*createDebugShape(Circle(pass.receiverPoint(), 0.03), std::to_string(debug_shapes.size()) + "s", stream.str()));
            if (rating > best_pass_for_receiving.rating)
            {
                best_pass_for_receiving = PassWithRating{pass, rating};
            }
        }
        std::stringstream stream;
        stream << std::fixed << std::setprecision(3) << best_pass_for_receiving.rating;
        debug_shapes.push_back(*createDebugShape(Circle(best_pass_for_receiving.pass.receiverPoint(), 0.05), std::to_string(debug_shapes.size()) + "s", stream.str()));

        best_receiving_positions.insert_or_assign(zone_id, best_pass_for_receiving);
    }
}
