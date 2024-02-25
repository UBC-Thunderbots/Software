#include "software/ai/evaluation/keep_away.h"

#include "software/ai/passing/cost_function.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/math/math_functions.h"
#include "software/optimization/gradient_descent_optimizer.hpp"

double ratePasserPointForKeepAway(const Pass& pass, const Team& enemy_team)
{
    // the default values for these passing parameters
    // TODO: cleanup passing parameters as part of #1987
    static constexpr double PASSER_ENEMY_PROXIMITY_IMPORTANCE   = 1.5;
    static constexpr double RECEIVER_ENEMY_PROXIMITY_IMPORTANCE = 0.;
    static const auto ENEMY_REACTION_TIME = Duration::fromSeconds(0);


    return ratePassEnemyRisk(enemy_team, pass, ENEMY_REACTION_TIME,
                             RECEIVER_ENEMY_PROXIMITY_IMPORTANCE) *
           (1 - std::max(0., calculateProximityRisk(pass.passerPoint(), enemy_team,
                                                    PASSER_ENEMY_PROXIMITY_IMPORTANCE)));
}


Point findKeepAwayTargetPoint(const WorldPtr& world_ptr, const Pass& best_pass_so_far)
{
    static constexpr auto KEEPAWAY_SEARCH_CIRCLE_RADIUS = 0.5;

    // the width of both the field boundary sigmoid and the circular search region sigmoid
    static constexpr auto SIGMOID_WIDTH = 0.1;
    // the inward offset of the field boundaries to use for the field lines sigmoid
    static constexpr auto FIELD_SIZE_REDUCTION_M = 0.25;

    static constexpr auto GRADIENT_STEPS_PER_ITER = 5;

    // the parameters for GradientDescentOptimizer were originally tuned for a 4 zone
    // field division. Since we are optimizing over a much smaller "search space",
    // we need to scale the gradient appropriately. Value empirically determined.
    static constexpr GradientDescentOptimizer<2>::ParamArray PARAM_WEIGHTS = {0.1, 0.1};


    // the region to which the optimization is (effectively) constrained to
    Circle keepaway_search_region(world_ptr->ball().position(),
                                  KEEPAWAY_SEARCH_CIRCLE_RADIUS);

    // a reduced field rectangle to effectively constrain the optimization to field
    // boundaries
    const auto& field_bounds       = world_ptr->field().fieldLines();
    Point reduced_bottom_left      = Point(field_bounds.xMin() + FIELD_SIZE_REDUCTION_M,
                                      field_bounds.yMin() + FIELD_SIZE_REDUCTION_M);
    Point reduced_top_right        = Point(field_bounds.xMax() - FIELD_SIZE_REDUCTION_M,
                                    field_bounds.yMax() - FIELD_SIZE_REDUCTION_M);
    Rectangle reduced_field_bounds = Rectangle(reduced_bottom_left, reduced_top_right);

    // the position rating function we want to maximize
    const auto keepaway_point_cost = [&](const std::array<double, 2>& passer_pt_array) {
        Point passer_pt(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
        Pass pass(passer_pt, best_pass_so_far.receiverPoint(), best_pass_so_far.speed());
        return ratePasserPointForKeepAway(pass, world_ptr->enemyTeam()) *
               // constrain the optimization to a circular area around the ball
               circleSigmoid(keepaway_search_region, passer_pt, SIGMOID_WIDTH) *
               // don't try to dribble the ball off the field
               rectangleSigmoid(reduced_field_bounds, passer_pt, SIGMOID_WIDTH);
    };
    GradientDescentOptimizer<2> optimizer{PARAM_WEIGHTS};
    auto passer_pt_array =
        optimizer.maximize(keepaway_point_cost,
                           std::array<double, 2>{world_ptr->ball().position().x(),
                                                 world_ptr->ball().position().y()},
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

bool shouldKeepAway(const Robot& robot, const Team& enemy_team,
                    double about_to_steal_radius)
{
    Circle about_to_steal_danger_zone(robot.position(), about_to_steal_radius);

    return std::any_of(enemy_team.getAllRobots().begin(), enemy_team.getAllRobots().end(),
                       [&](const auto& enemy) {
                           return contains(about_to_steal_danger_zone, enemy.position());
                       });
}
