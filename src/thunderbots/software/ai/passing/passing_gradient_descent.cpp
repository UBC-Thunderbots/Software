#include <algorithm>
#include <numeric>
#include "ai/passing/passing_gradient_descent.h"
#include "passing_gradient_descent.h"


using namespace AI::Passing;

GradientDescent::GradientDescent() :
num_gradient_descent_paths(50),
gradient_approx_step_size(0.001),
min_reasonable_pass_quality(0),
// Init all param weights to 1
param_weights({ 0.1, 0.1, 1, 1 }),
past_gradient_decay_rate(0.9),
past_squared_gradient_decay_rate(0.999)
{
    passes_to_optimize = generatePaths(num_gradient_descent_paths);
}

void GradientDescent::setWorld(World world) {
    this->world = world;
}

void GradientDescent::setPasserPoint(Point passer_point) {
    this->passer_point = passer_point;
}

void GradientDescent::runGradientDescent(unsigned int num_iterations) {

    // Run gradient descent on each pass for the requested number of iterations
    for (GradientDescentPass& pass: passes_to_optimize){
        for (unsigned int i = 0; i < num_iterations; i++){
            pass = optimizePass(pass);
        }
    }

    // TODO: re-add me!
    //// Prune the least promising 1/2 of the paths
    //std::sort(passes_to_optimize.begin(), passes_to_optimize.end(),
    //        [this](auto pass1, auto pass2) {return comparePassQuality(pass1, pass2);}
    //        );
    //passes_to_optimize = std::vector<GradientDescentPass>(passes_to_optimize.begin(),
    //        passes_to_optimize.begin() + passes_to_optimize.size()/2);

    //// Replace the paths we just removed
    //unsigned long num_paths_needed = num_gradient_descent_paths - passes_to_optimize.size();
    //std::vector<GradientDescentPass> new_paths = generatePaths(num_paths_needed);
    //passes_to_optimize.insert(passes_to_optimize.end(), new_paths.begin(), new_paths.end());
}

std::optional<Pass> GradientDescent::getBestPass() {
    std::sort(passes_to_optimize.begin(), passes_to_optimize.end(),
              [this](auto pass1, auto pass2) {return comparePassQuality(pass1, pass2);}
    );
    if (passes_to_optimize.size() > 0 && ratePass(passes_to_optimize[0]) > min_reasonable_pass_quality){
        return std::optional(passes_to_optimize[0]);
    }
    return std::optional<Pass>();
}

GradientDescent::GradientDescentPass GradientDescent::optimizePass(GradientDescentPass pass) {
    // Implementation of the "Adam" algorithm
    // ( https://en.wikipedia.org/wiki/Stochastic_gradient_descent#Adam )
    // ( http://ruder.io/optimizing-gradient-descent/index.html#adam )

    // TODO: is this minimizing or maximizing??

    // Approximate the gradient of all the parameters around the current pass
    std::array<double, NUM_OPTIMIZE_PARAMS> gradient = approximateGradient(pass);

    // Get the squared gradient
    std::array<double, NUM_OPTIMIZE_PARAMS> squared_gradient = { 0 };
    for (unsigned int i = 0; i < NUM_OPTIMIZE_PARAMS; i++){
        squared_gradient[i] = std::pow(gradient[i], 2);
    }

    // Update past gradient and gradient squared averages
    for (unsigned int i = 0; i < NUM_OPTIMIZE_PARAMS; i++){
        pass.past_gradient_averages[i] = past_gradient_decay_rate *
                pass.past_gradient_averages[i]  + (1 - past_gradient_decay_rate) *
                gradient[i];
        pass.past_squared_gradient_averages[i] = past_squared_gradient_decay_rate *
                pass.past_squared_gradient_averages[i]  + (1 - past_squared_gradient_decay_rate) *
                squared_gradient[i];
    }

    // Create the bias corrected gradient and gradient square averages
    std::array<double, NUM_OPTIMIZE_PARAMS> bias_corrected_past_gradient_averages = { 0 };
    std::array<double, NUM_OPTIMIZE_PARAMS> bias_corrected_past_squared_gradient_averages = { 0 };
    for (unsigned int i = 0; i < NUM_OPTIMIZE_PARAMS; i++){
        bias_corrected_past_gradient_averages[i] = pass.past_gradient_averages[i]
                / (1 - std::pow(past_gradient_decay_rate, 2));
        bias_corrected_past_squared_gradient_averages[i] =
                pass.past_squared_gradient_averages[i]
                / (1 - std::pow(past_squared_gradient_decay_rate, 2));
    }

    // Update the parameter values
    std::array<double, NUM_OPTIMIZE_PARAMS> params = pass.getParams();
    for (unsigned int i = 0; i < NUM_OPTIMIZE_PARAMS; i++){
        double step_size = param_weights[i] *
                bias_corrected_past_gradient_averages[i] /
                (std::sqrt(bias_corrected_past_squared_gradient_averages[i]) + eps);
        params[i] = params[i] + param_weights[i] *
                bias_corrected_past_gradient_averages[i] /
                (std::sqrt(bias_corrected_past_squared_gradient_averages[i]) + eps);
    }

    return GradientDescentPass(params);
}

std::array<double, NUM_OPTIMIZE_PARAMS> GradientDescent::approximateGradient(GradientDescentPass pass) {

    std::array<double, NUM_OPTIMIZE_PARAMS> gradient = { 0 };
    double curr_pass_quality = ratePass(pass);
    for (int i = 0; i < NUM_OPTIMIZE_PARAMS; i++){
        auto test_params = pass.getParams();
        test_params[i] += gradient_approx_step_size * param_weights[i];
        GradientDescentPass new_pass(test_params);
        gradient[i] = (ratePass(new_pass) - curr_pass_quality) / gradient_approx_step_size;
    }

    return gradient;
}

double GradientDescent::ratePass(GradientDescent::GradientDescentPass pass) {

    double pass_quality = getStaticPositionQuality(world.field(), pass.receiver_point);

    double distance_to_goal = Vector(pass.receiver_point.x() - world.field().enemyGoal().x(),
            pass.receiver_point.y() - world.field().enemyGoal().y()).len();

    pass_quality = 1 / (1 + std::exp(distance_to_goal - 2.1));

    // TODO: the rest of this function; see the old code

    return pass_quality;
}

std::vector<GradientDescent::GradientDescentPass> GradientDescent::generatePaths(unsigned long num_paths_to_gen) {
    GradientDescentPass pass_to_center;
    pass_to_center.pass_start_time = world.ball().lastUpdateTimestamp();
    pass_to_center.receiver_point = Point(4, 0);
    pass_to_center.pass_speed_m_per_s = 2;

    // TODO: implement this function properly; see the old code

    return std::vector<GradientDescentPass>(num_paths_to_gen, pass_to_center);
}

bool GradientDescent::comparePassQuality(GradientDescent::GradientDescentPass pass1, GradientDescent::GradientDescentPass pass2) {
    return ratePass(pass1) < ratePass(pass2);
}
