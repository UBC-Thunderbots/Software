#include "software/ai/evaluation/keep_away.h"

#include "software/ai/passing/cost_function.h"
#include "software/math/math_functions.h"
#include "software/optimization/gradient_descent_optimizer.h"

double ratePasserPointForKeepAway(const Pass& pass, const Team& enemy_team)
{
    // the default values for these passing parameters
    // TODO: cleanup passing parameters as part of #1987
    // TODO: tweak all of these constants to get better behaviour
    static constexpr double ENEMY_PROXIMITY_IMPORTANCE = 1.5;
    static const auto ENEMY_REACTION_TIME              = Duration::fromSeconds(0);


    return ratePassEnemyRisk(enemy_team, pass, ENEMY_REACTION_TIME, 0) *
           (1 - calculateProximityRisk(pass.passerPoint(), enemy_team,
                                       ENEMY_PROXIMITY_IMPORTANCE));
}


Point findKeepAwayTargetPoint(const Point& ball_possessor_position,
                              const Pass& best_pass_so_far, const World& world)
{
    // TODO: tweak this constant
    static constexpr auto KEEPAWAY_SEARCH_CIRCLE_RADIUS = 0.5;

    // the width of both the field boundary sigmoid and the circular search region sigmoid
    static constexpr auto SIGMOID_WIDTH = 0.05;

    static constexpr auto GRADIENT_STEPS_PER_ITER = 2;

    // the region to which the optimization is (effectively) constrained to
    Circle keepaway_search_region(world.ball().position(), KEEPAWAY_SEARCH_CIRCLE_RADIUS);

    // the position rating function we want to maximize
    // TODO: visualize this shit
    const auto keepaway_point_cost = [&](const std::array<double, 2>& passer_pt_array) {
        Point passer_pt(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
        Pass pass(passer_pt, best_pass_so_far.receiverPoint(), best_pass_so_far.speed());
        return ratePasserPointForKeepAway(pass, world.enemyTeam()) *
               circleSigmoid(keepaway_search_region, passer_pt, SIGMOID_WIDTH) *
               rectangleSigmoid(world.field().fieldLines(), passer_pt, SIGMOID_WIDTH);
    };
    GradientDescentOptimizer<2> optimizer{{0.1, 0.1}};
    auto passer_pt_array = optimizer.maximize(
        keepaway_point_cost,
        std::array<double, 2>{ball_possessor_position.x(), ball_possessor_position.y()},
        GRADIENT_STEPS_PER_ITER);
    return Point(std::get<0>(passer_pt_array), std::get<1>(passer_pt_array));
}
