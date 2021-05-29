#include "software/ai/evaluation/keep_away.h"

#include "software/ai/passing/cost_function.h"
#include "software/math/math_functions.h"
#include "software/optimization/gradient_descent_optimizer.h"


Point findKeepAwayTargetPoint(const Point& ball_possessor_position,
                              const Pass& best_pass_so_far, const World& world)
{
    // the theory of this keepaway point is that the below constant is essentially the
    // "step size" and the robot should always move in a "descent direction" until
    // the play determines that we can pass, or we reach a local optimum of the
    // enemy risk function
    // the below constant must be set such that this "optimization" "converges"
    static constexpr auto KEEPAWAY_SEARCH_CIRCLE_RADIUS = 0.3;
    static constexpr auto SIGMOID_WIDTH                 = 0.1;

    // the default values for these passing parameters
    // TODO: cleanup passing parameters as part of #1987
    static constexpr double ENEMY_PROXIMITY_IMPORTANCE = 0.5;
    static const auto ENEMY_REACTION_TIME              = Duration::fromSeconds(0.4);
    static constexpr auto GRADIENT_STEPS_PER_ITER      = 2;

    // the region to which the optimization is (effectively) constrained to
    Circle keepaway_search_region(ball_possessor_position, KEEPAWAY_SEARCH_CIRCLE_RADIUS);

    // the position rating function we want to maximize
    // TODO: rectangle sigmoid for field bounds
    const auto keepaway_point_cost = [&](const std::array<double, 2>& passer_pt_array) {
        Point passer_pt(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
        Pass pass(passer_pt, best_pass_so_far.receiverPoint(), best_pass_so_far.speed());
        return ratePassEnemyRisk(world.enemyTeam(), pass, ENEMY_REACTION_TIME,
                                 ENEMY_PROXIMITY_IMPORTANCE) *
               circleSigmoid(keepaway_search_region, passer_pt, SIGMOID_WIDTH) *
               rectangleSigmoid(world.field().fieldLines(), passer_pt, SIGMOID_WIDTH);
    };
    GradientDescentOptimizer<2> optimizer;
    auto passer_pt_array = optimizer.maximize(
        keepaway_point_cost,
        std::array<double, 2>{ball_possessor_position.x(), ball_possessor_position.y()},
        GRADIENT_STEPS_PER_ITER);
    return Point(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
}
