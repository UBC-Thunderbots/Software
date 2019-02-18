//
// Created by gareth on 17/02/19.
//

#include <algorithm>
#include "passing_gradient_descent.h"

PassingGradientDescent::PassingGradientDescent(unsigned int num_gradient_descent_points) :
num_gradient_descent_points(num_gradient_descent_points)
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

void PassingGradientDescent::optimizePass(GradientDescentPass pass, unsigned int num_iterations) {
    // find derivative of all the params
    std::array<double, NUM_OPTIMIZE_PARAMS> = approximateGradient(pass);
    grad = approximateGradient(
            snapshot, current_params, tiny_step, current_func_val, num_params,
            weights);
    func_evals = func_evals + num_params;

    // step the test params
    test_params = current_params;

    norm_grad = 0;
    for (unsigned int i = 0; i < num_params; i++)
    {
        norm_grad += grad[i] * grad[i];
    }
    norm_grad = std::pow(norm_grad, 0.5);
    for (unsigned int i = 0; i < num_params; i++)
    {
        // step: new = old + learningConst*weight*grad of this param/length
        // of grad of all params
        test_params[i] =
                test_params[i] + alpha * weights[i] * grad[i] / norm_grad;
    }

    // calculate the test func val
    test_func_val = ratePass(
            snapshot, Point(test_params[0], test_params[1]), test_params[2],
            test_params[3]);

    // if difference is large enough, increase the learning constant
    if ((test_func_val - current_func_val) > 0)
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

std::array<double, NUM_OPTIMIZE_PARAMS> PassingGradientDescent::approximateGradient(
        GradientDescentPass pass, double step_size) {

    std::array<double, NUM_OPTIMIZE_PARAMS> gradient = { 0 };
    double curr_pass_quality = ratePass(pass);
    for (int i = 0; i < NUM_OPTIMIZE_PARAMS; i++){
        auto test_params = pass.getParams();
        test_params[i] += step_size * param_weights[i];
        GradientDescentPass new_pass(test_params);
        gradient[i] = ratePass(new_pass) - curr_pass_quality;
    }

    return gradient;
}
