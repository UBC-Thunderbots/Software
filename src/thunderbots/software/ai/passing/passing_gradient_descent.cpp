#include <algorithm>
#include <numeric>
#include "ai/passing/passing_gradient_descent.h"
#include "passing_gradient_descent.h"
#include "ai/passing/evaluation.h"


using namespace AI::Passing;


PassGenerator::PassGenerator(double min_reasonable_pass_quality):
min_reasonable_pass_quality(min_reasonable_pass_quality),
optimizer(optimizer_param_weights)
{
    passes_to_optimize = generatePasses(this->num_passes_to_optimize);
}

void PassGenerator::setWorld(World world) {
    this->world = std::move(world);
}

void PassGenerator::setPasserPoint(Point passer_point) {
    this->passer_point = passer_point;
}

std::optional<Pass> PassGenerator::getBestPass() {
    std::sort(passes_to_optimize.begin(), passes_to_optimize.end(),
              [this](auto pass1, auto pass2) {return comparePassQuality(pass1, pass2);}
    );
    if (!passes_to_optimize.empty() && ratePass(passes_to_optimize[0]) > min_reasonable_pass_quality){
        return std::optional(passes_to_optimize[0]);
    }

    return std::nullopt;
}

void PassGenerator::iterate() {

    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    static const auto objective_function =
            [this](std::array<double, NUM_PARAMS_TO_OPTIMIZE> pass_array){
                Pass pass = convertArrayToPass(pass_array);
                return ratePass(pass);
            };

    // Run gradient descent to optimize the passes to for the requested number
    // of iterations
    // TODO: Can we parallize this? Would it make it faster?
    for (Pass& pass : passes_to_optimize){
        auto pass_array = optimizer.maximize(
                objective_function,
                convertPassToArray(pass),
                number_of_gradient_descent_steps_per_iter
                );
        pass = convertArrayToPass(pass_array);
    }

    // Replace un-promising passes with new passes
    std::sort(passes_to_optimize.begin(), passes_to_optimize.end(),
            [this](Pass p1, Pass p2) { return comparePassQuality(p1, p2); }
            );
    if (num_passes_to_keep_after_pruning < num_passes_to_optimize){
        std::vector<Pass> new_passes = generatePasses(num_passes_to_optimize -
                                                      num_passes_to_keep_after_pruning);
        std::move(new_passes.begin(), new_passes.end(), passes_to_optimize.begin() + num_passes_to_keep_after_pruning);
    }
}

double PassGenerator::ratePass(Pass pass) {

    double pass_quality = getStaticPositionQuality(world.field(), pass.receiverPoint());

    double distance_to_goal = Vector(pass.receiverPoint().x() - world.field().enemyGoal().x(),
            pass.receiverPoint().y() - world.field().enemyGoal().y()).len();

    pass_quality = 1 / (1 + std::exp(distance_to_goal - 2.1));

    // TODO: the rest of this function; see the old code

    return pass_quality;
}

std::vector<Pass> PassGenerator::generatePasses(unsigned long num_paths_to_gen) {
    Pass pass_to_center;
    pass_to_center.passStartTime() = world.ball().lastUpdateTimestamp();
    pass_to_center.receiverPoint() = Point(4, 0);
    pass_to_center.passSpeed() = 2;

    // TODO: implement this function properly; see the old code

    return std::vector<Pass>(num_paths_to_gen, pass_to_center);
}

bool PassGenerator::comparePassQuality(Pass pass1, Pass pass2) {
    return ratePass(std::move(pass1)) < ratePass(std::move(pass2));
}

std::array<double, PassGenerator::NUM_PARAMS_TO_OPTIMIZE> PassGenerator::convertPassToArray(Pass pass) {
    return {
        pass.receiverPoint().x(),
        pass.receiverPoint().y(),
        pass.passSpeed(),
        pass.passStartTime().getSeconds()
    };
}

Pass PassGenerator::convertArrayToPass(std::array<double, PassGenerator::NUM_PARAMS_TO_OPTIMIZE> array) {
    return Pass(passer_point, Point(array.at(0), array.at(1)), array.at(2),
            Timestamp::fromSeconds(array.at(3)));
}
