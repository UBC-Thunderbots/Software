#include "software/ai/evaluation/keep_away.h"

#include "software/ai/passing/cost_function.h"
#include "software/math/math_functions.h"
#include "software/optimization/gradient_descent_optimizer.h"


Point findKeepAwayTargetPoint(const Point& ball_possessor_position,
                              const Pass& best_pass_so_far, const World& world,
                              std::shared_ptr<const PassingConfig> passing_config)
{
    static constexpr auto KEEPAWAY_SEARCH_CIRCLE_RADIUS = 0.3;
    static constexpr auto CIRCLE_SIGMOID_WIDTH          = 0.1;

    // the region to which the optimization is (effectively) constrained to
    Circle keepaway_search_region(ball_possessor_position, KEEPAWAY_SEARCH_CIRCLE_RADIUS);

    // the position rating function we want to maximize
    // TODO: rectangle sigmoid for field bounds
    const auto keepaway_point_cost = [&](const std::array<double, 2>& passer_pt_array) {
        Point passer_pt(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
        Pass pass(passer_pt, best_pass_so_far.receiverPoint(), best_pass_so_far.speed());
        return ratePassEnemyRisk(world.enemyTeam(), pass, passing_config) *
               circleSigmoid(keepaway_search_region, passer_pt, CIRCLE_SIGMOID_WIDTH);
    };
    GradientDescentOptimizer<2> optimizer;
    auto passer_pt_array = optimizer.maximize(
        keepaway_point_cost,
        std::array<double, 2>{ball_possessor_position.x(), ball_possessor_position.y()},
        passing_config->getNumberOfGradientDescentStepsPerIter()->value());
    return Point(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
}
