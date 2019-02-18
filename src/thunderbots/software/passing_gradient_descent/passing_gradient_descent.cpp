//
// Created by gareth on 17/02/19.
//

#include <algorithm>
#include <numeric>
#include "passing_gradient_descent.h"

using namespace PassingGradientDescent;

PassingGradientDescent::PassingGradientDescent(unsigned int num_gradient_descent_points, double gradient_approx_step_size) :
num_gradient_descent_points(num_gradient_descent_points),
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

    // Run gradient descent for the requested number of iterations
    for (GradientDescentPass& pass: passes_to_optimize){
        optimizePass(pass, num_iterations);
    }

    // Prune the least promising points and replace them with random points
}

std::optional<Pass> PassingGradientDescent::getBestPass() {
    // TODO
    return std::optional<Pass>();
}

PassingGradientDescent::GradientDescentPass PassingGradientDescent::optimizePass(GradientDescentPass pass, unsigned int num_iterations) {
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
        params[i] = params[i] - param_weights[i]
    }


    // Update averages for past squared gradients

    // Calculate the L2 norm (Euclidean distance)
    double gradient_norm =
            std::pow(
                std::accumulate(gradient.begin(), gradient.end(), 0.0,
                [](double sum, double grad){ return sum + grad * grad; }),
                0.5);

    std::array<double, NUM_OPTIMIZE_PARAMS> test_params = pass.getParams();

    for (unsigned int i = 0; i < NUM_OPTIMIZE_PARAMS; i++)
    {
        // Step: new = old + learningConst*weight*grad of this param/
        // (length of grad of all params)
        test_params[i] =
                test_params[i] + pass.getLearningConstant() * param_weights[i] * gradient[i] / gradient_norm;
    }

    // Calculate the quality of the pass after we've stepped
    double new_pass_quality = ratePass(Pass(test_params));

    // If we improved, increase the learning constant
    if ((new_pass_quality - curr_pass_quality) > 0)
    {
        alpha = 1.3 * alpha;
        // std::cout << "successful point: " << test_func_val << std::endl;
    }

    else
    {
        while ((test_func_val - current_func_val) < 0 &&
               func_evals <= max_func_evals)
        {
            alpha = 0.5 * alpha;
            // std::cout << "failed point: " << test_func_val << "   alpha:
            // " << alpha << "   norm grad: " << norm_grad << std::endl;
            test_params = current_params;
            for (unsigned int i = 0; i < num_params; i++)
            {
                test_params[i] = test_params[i] +
                                 alpha * weights[i] * grad[i] / norm_grad;
            }
            test_func_val = ratePass(
                    snapshot, Point(test_params[0], test_params[1]),
                    test_params[2], test_params[3]);
            func_evals++;
        }
    }

    // if the algorithm helped, keep the new values
    if (test_func_val > current_func_val)
    {
        current_params   = test_params;
        current_func_val = test_func_val;
    }
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
