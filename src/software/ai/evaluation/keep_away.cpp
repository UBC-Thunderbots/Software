#include "software/ai/evaluation/keep_away.h"

#include "software/ai/passing/cost_function.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/math/math_functions.h"
#include "software/optimization/gradient_descent_optimizer.hpp"

Point findKeepAwayTargetPoint(const World& world, const Pass& best_pass_so_far)
{
    // the inward offset of the field boundaries to use for the field lines sigmoid
    static constexpr auto FIELD_SIZE_REDUCTION_M = 0.25;

    static constexpr auto GRADIENT_STEPS_PER_ITER = 5;

    // the parameters for GradientDescentOptimizer were originally tuned for a 4 zone
    // field division. Since we are optimizing over a much smaller "search space",
    // we need to scale the gradient appropriately. Value empirically determined.
    static constexpr GradientDescentOptimizer<2>::ParamArray PARAM_WEIGHTS = {0.1, 0.1};

    // a reduced field rectangle to effectively constrain the optimization to field
    // boundaries
    const auto& field_bounds       = world.field().fieldLines();
    Point reduced_bottom_left      = Point(field_bounds.xMin() + FIELD_SIZE_REDUCTION_M,
                                      field_bounds.yMin() + FIELD_SIZE_REDUCTION_M);
    Point reduced_top_right        = Point(field_bounds.xMax() - FIELD_SIZE_REDUCTION_M,
                                    field_bounds.yMax() - FIELD_SIZE_REDUCTION_M);
    Rectangle reduced_field_bounds = Rectangle(reduced_bottom_left, reduced_top_right);

    // the position rating function we want to maximize
    const auto keepaway_point_cost = [&](const std::array<double, 2>& passer_pt_array) {
        Point passer_pt(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
        return ratePasserPosition(
            world,
            Pass(passer_pt, best_pass_so_far.receiverPoint(), best_pass_so_far.speed()),
            reduced_field_bounds);
    };
    GradientDescentOptimizer<2> optimizer{PARAM_WEIGHTS};
    auto passer_pt_array = optimizer.maximize(
        keepaway_point_cost,
        std::array<double, 2>{world.ball().position().x(), world.ball().position().y()},
        GRADIENT_STEPS_PER_ITER);
    Point keepaway_target_point(std::get<0>(passer_pt_array),
                                std::get<1>(passer_pt_array));

    if (!contains(reduced_field_bounds, keepaway_target_point))
    {
        // the point reached by the optimization is outside the reduced field boundaries,
        // project it onto the reduced field boundaries rectangle and return that
        const auto& field_line_segments = reduced_field_bounds.getSegments();
        std::vector<std::pair<Point, double>> closest_points_and_distances(
            field_line_segments.size());
        std::transform(field_line_segments.begin(), field_line_segments.end(),
                       closest_points_and_distances.begin(),
                       [keepaway_target_point](const Segment& seg) {
                           auto closest_point = closestPoint(keepaway_target_point, seg);
                           return std::make_pair(
                               closest_point,
                               (keepaway_target_point - closest_point).length());
                       });
        const auto& closest_field_line_seg_and_dist = *std::min_element(
            closest_points_and_distances.begin(), closest_points_and_distances.end(),
            [](const auto& lhs_point_and_dist, const auto& rhs_point_and_dist) {
                // compare the distances
                return lhs_point_and_dist.second < rhs_point_and_dist.second;
            });
        return closest_field_line_seg_and_dist.first;
    }
    else
    {
        // the point reached by the optimization is inside the field boundaries
        return keepaway_target_point;
    }
}
