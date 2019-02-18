//
// Created by gareth on 17/02/19.
//

#include <algorithm>
#include <numeric>
#include "passing_gradient_descent.h"

using namespace PassingGradientDescent;

PassingGradientDescent::PassingGradientDescent(unsigned int num_gradient_descent_points, double gradient_approx_step_size) :
num_gradient_descent_paths(num_gradient_descent_points),
gradient_approx_step_size(gradient_approx_step_size)
{

}

void PassingGradientDescent::setWorld(World world) {
    world = world;
}

void PassingGradientDescent::setPasserPoint(Point passer_point) {
    passer_point = passer_point;
}

void PassingGradientDescent::runGradientDescent(unsigned int num_iterations) {

    // Run gradient descent on each pass for the requested number of iterations
    for (GradientDescentPass& pass: passes_to_optimize){
        for (unsigned int i = 0; i < num_iterations; i++){
            pass = optimizePass(pass);
        }
    }

    // Prune the least promising 1/2 of the paths
    std::sort(passes_to_optimize.begin(), passes_to_optimize.end());
    passes_to_optimize = std::vector<GradientDescentPass>(passes_to_optimize.begin(),
            passes_to_optimize.begin() + passes_to_optimize.size()/2);

    // Replace the paths we just removed
    unsigned int num_paths_needed = num_gradient_descent_paths - passes_to_optimize.size();
    std::vector<GradientDescentPass> new_paths = generatePaths(num_paths_needed);
    passes_to_optimize.insert(passes_to_optimize.end(), new_paths.begin(), new_paths.end());
}

std::optional<Pass> PassingGradientDescent::getBestPass() {
    std::sort(passes_to_optimize.begin(), passes_to_optimize.end());
    if (passes_to_optimize.size() > 0 && ratePass(passes_to_optimize[0]) > min_reasonable_pass_quality){
        return std::optional(passes_to_optimize[0]);
    }
    return std::optional<Pass>();
}

PassingGradientDescent::GradientDescentPass PassingGradientDescent::optimizePass(GradientDescentPass pass) {
    // Implementation of the "Adam" algorithm
    // ( https://en.wikipedia.org/wiki/Stochastic_gradient_descent#Adam )
    // ( http://ruder.io/optimizing-gradient-descent/index.html#adam )

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
    std::array<double, NUM_OPTIMIZE_PARAMS> bias_corrected_past_square_gradient_averages = { 0 };
    for (unsigned int i = 0; i < NUM_OPTIMIZE_PARAMS; i++){
        bias_corrected_past_gradient_averages[i] = pass.past_gradient_averages[i]
                / (1 - std::pow(past_gradient_decay_rate, 2));
        bias_corrected_past_square_gradient_averages[i] =
                pass.past_squared_gradient_averages[i]
                / (1 - std::pow(past_squared_gradient_decay_rate, 2));
    }

    // Update the parameter values
    std::array<double, NUM_OPTIMIZE_PARAMS> params = pass.getParams();
    for (unsigned int i = 0; i < NUM_OPTIMIZE_PARAMS; i++){
        params[i] = params[i] - param_weights[i] *
                bias_corrected_past_gradient_averages[i] /
                (std::sqrt(bias_corrected_past_square_gradient_averages[i]) + eps);
    }

    return GradientDescentPass(params);
}

std::array<double, NUM_OPTIMIZE_PARAMS> PassingGradientDescent::approximateGradient(GradientDescentPass pass) {

    std::array<double, NUM_OPTIMIZE_PARAMS> gradient = { 0 };
    double curr_pass_quality = ratePass(pass);
    for (int i = 0; i < NUM_OPTIMIZE_PARAMS; i++){
        auto test_params = pass.getParams();
        test_params[i] += gradient_approx_step_size * param_weights[i];
        GradientDescentPass new_pass(test_params);
        gradient[i] = ratePass(new_pass) - curr_pass_quality;
    }

    return gradient;
}
