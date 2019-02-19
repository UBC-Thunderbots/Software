#include <algorithm>
#include <numeric>
#include "ai/passing/passing_gradient_descent.h"
#include "passing_gradient_descent.h"


using namespace AI::Passing;

PassGenerator::PassGenerator() :
num_gradient_descent_passes(50),
min_reasonable_pass_quality(0),
// Init all param weights to 1
optimizer(
        0.001,
        { 0.1, 0.1, 1, 1 },
        0.9,
        0.999
        )
{
    passes_to_optimize = generatePasses(num_gradient_descent_passes);
}

void PassGenerator::setWorld(World world) {
    this->world = world;
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

}

double PassGenerator::ratePass(std::array<double, Pass::NUM_PARAMS> params) {
    return ratePass(Pass(params));
}

double PassGenerator::ratePass(Pass pass) {

    double pass_quality = getStaticPositionQuality(world.field(), pass.receiver_point);

    double distance_to_goal = Vector(pass.receiver_point.x() - world.field().enemyGoal().x(),
            pass.receiver_point.y() - world.field().enemyGoal().y()).len();

    pass_quality = 1 / (1 + std::exp(distance_to_goal - 2.1));

    // TODO: the rest of this function; see the old code

    return pass_quality;
}

std::vector<Pass> PassGenerator::generatePasses(unsigned long num_paths_to_gen) {
    Pass pass_to_center;
    pass_to_center.pass_start_time = world.ball().lastUpdateTimestamp();
    pass_to_center.receiver_point = Point(4, 0);
    pass_to_center.pass_speed_m_per_s = 2;

    // TODO: implement this function properly; see the old code

    return std::vector<Pass>(num_paths_to_gen, pass_to_center);
}

bool PassGenerator::comparePassQuality(Pass pass1, Pass pass2) {
    return ratePass(pass1) < ratePass(pass2);
}
