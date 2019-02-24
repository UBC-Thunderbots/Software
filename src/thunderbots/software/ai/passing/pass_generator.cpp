#include "ai/passing/pass_generator.h"

#include <algorithm>
#include <numeric>

#include "ai/passing/evaluation.h"
#include "pass_generator.h"


using namespace AI::Passing;
using namespace Util::DynamicParameters::AI::Passing;

PassGenerator::PassGenerator(double min_reasonable_pass_quality)
    : min_reasonable_pass_quality(min_reasonable_pass_quality),
      optimizer(optimizer_param_weights),
      best_known_pass(std::nullopt),
      target_region(std::nullopt),
      in_destructor(false)
{
    // Generate the initial set of passes
    passes_to_optimize = generatePasses(num_passes_to_optimize.value());

    // Start the thread to do the pass generation in the background
    // The lambda expression here is needed so that we can call
    // `continuouslyGeneratePasses()`, which is not a static function
    pass_generation_thread =
        std::thread([this]() { return continuouslyGeneratePasses(); });
}

void PassGenerator::setWorld(World world)
{
    // Take ownership of the world for the duration of this function
    std::lock_guard<std::mutex> world_lock(world_mutex);

    // Update the world
    this->world = std::move(world);
}

void PassGenerator::setPasserPoint(Point passer_point)
{
    // Take ownership of the passer_point for the duration of this function
    std::lock_guard<std::mutex> passer_point_lock(passer_point_mutex);

    // Update the passer point
    this->passer_point = passer_point;

    // Replace all the passes we're currently trying to optimize, as they are probably
    // not going to converge to this new passer point if they've already been converging
    // to another passer point for a while
    passes_to_optimize = generatePasses(num_passes_to_optimize.value());
}

std::optional<Pass> PassGenerator::getBestPassSoFar()
{
    // Take ownership of the best_known_pass for the duration of this function
    std::lock_guard<std::mutex> best_known_pass_lock(best_known_pass_mutex);

    return best_known_pass;
}

void PassGenerator::setTargetRegion(std::optional<Rectangle> area)
{
    // Take ownership of the target_region for the duration of this function
    std::lock_guard<std::mutex> target_region_lock(target_region_mutex);

    this->target_region = std::move(area);
}


PassGenerator::~PassGenerator()
{
    // Set this flag so pass_generation_thread knows to end (also making sure to
    // properly take and give ownership of the flag)
    in_destructor_mutex.lock();
    in_destructor = true;
    in_destructor_mutex.unlock();

    // Join to pass_generation_thread so that we wait for it to exit before destructing
    // the thread object. If we do not wait for thread to finish executing, it will
    // call `std::terminate` when we deallocate the thread object and kill our whole
    // program
    pass_generation_thread.join();
}

void PassGenerator::continuouslyGeneratePasses()
{
    // Take ownership of the in_destructor flag so we can use it for the conditional
    // check
    in_destructor_mutex.lock();
    while (!in_destructor)
    {
        // Give up ownership of the in_destructor flag now that we're done the
        // conditional check
        in_destructor_mutex.unlock();

        optimizePasses();
        pruneAndReplacePasses();
        saveBestPass();

        // Yield to allow other threads to run. This is particularly important if we
        // have this thread and another running on one core
        std::this_thread::yield();

        // Take ownership of the `in_destructor` flag so we can use it for the conditional
        // check
        in_destructor_mutex.lock();
    }
}

void PassGenerator::optimizePasses()
{
    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    const auto objective_function =
        [this](std::array<double, NUM_PARAMS_TO_OPTIMIZE> pass_array) {
            Pass pass = convertArrayToPass(pass_array);
            return ratePass(pass);
        };

    // Run gradient descent to optimize the passes to for the requested number
    // of iterations
    // NOTE: Parallelizing this `for` loop would probably be safe and potentially more
    //       performant
    for (Pass& pass : passes_to_optimize)
    {
        auto pass_array =
            optimizer.maximize(objective_function, convertPassToArray(pass),
                               number_of_gradient_descent_steps_per_iter.value());
        pass = convertArrayToPass(pass_array);
    }
}

void PassGenerator::pruneAndReplacePasses()
{
    // Sort the passes by increasing quality
    std::sort(passes_to_optimize.begin(), passes_to_optimize.end(),
              [this](Pass p1, Pass p2) { return comparePassQuality(p1, p2); });

    // Merge Passes That Are Similar
    // We start by assuming that the most similar passes will be right beside each other,
    // then iterate over the entire list, building a new list as we go by only adding
    // elements when they are dissimilar enough from the last element we added
    passes_to_optimize = std::accumulate(
        passes_to_optimize.begin(), passes_to_optimize.end(), std::vector<Pass>(),
        [this](std::vector<Pass>& passes, Pass curr_pass) {
            // Check if we have no passes, or if this pass is too similar to the
            // last pass we added to the list
            if (passes.empty() || !passesEqual(curr_pass, passes.back()))
            {
                passes.emplace_back(curr_pass);
            }
            return passes;
        });
    // Reverse the list to sort by increasing quality
    std::reverse(passes_to_optimize.begin(), passes_to_optimize.end());

    // Replace the least promising passes with newly generated passes
    if (num_passes_to_keep_after_pruning.value() < num_passes_to_optimize.value())
    {
        // Remove the worst paths
        if (num_passes_to_keep_after_pruning.value() < passes_to_optimize.size())
        {
            passes_to_optimize.erase(
                passes_to_optimize.begin() + num_passes_to_keep_after_pruning.value(),
                passes_to_optimize.end());
        }

        // Generate new passes to replace the ones we just removed
        std::vector<Pass> new_passes =
            generatePasses(num_passes_to_optimize.value() - passes_to_optimize.size());

        // Append our newly generated passes to replace the passes we just removed
        passes_to_optimize.insert(passes_to_optimize.end(), new_passes.begin(),
                                  new_passes.end());
    }
}

void PassGenerator::saveBestPass()
{
    // Take ownership of the best_known_pass for the duration of this function
    std::lock_guard<std::mutex> best_known_pass_lock(best_known_pass_mutex);

    // Sort the passes by decreasing quality
    std::sort(
        passes_to_optimize.begin(), passes_to_optimize.end(),
        [this](auto pass1, auto pass2) { return comparePassQuality(pass1, pass2); });
    if (!passes_to_optimize.empty() &&
        ratePass(passes_to_optimize[0]) >= min_reasonable_pass_quality)
    {
        best_known_pass = std::optional(passes_to_optimize[0]);
    }
    else
    {
        best_known_pass = std::nullopt;
    }
}

double PassGenerator::ratePass(Pass pass)
{
    // Take ownership of world, target_region for the duration of this function
    std::lock_guard<std::mutex> world_lock(world_mutex);
    std::lock_guard<std::mutex> target_region_lock(target_region_mutex);

    double static_pass_quality =
        getStaticPositionQuality(world.field(), pass.receiverPoint());

    // TODO (Issue #383): Implement this properly

    // Strongly weight positions in our target region, if we have one
    double in_region_quality = 1;
    if (target_region)
    {
        in_region_quality = rectangleSigmoid(*target_region, pass.receiverPoint(), 0.1);
    }

    return static_pass_quality * in_region_quality;
}

std::vector<Pass> PassGenerator::generatePasses(unsigned long num_paths_to_gen)
{
    Pass p(Point(0, 0), Point(0, 0), 0, Timestamp::fromSeconds(0));

    // TODO (Issue #382): Implement this properly

    return std::vector<Pass>(num_paths_to_gen, p);
}

bool PassGenerator::comparePassQuality(const Pass& pass1, const Pass& pass2)
{
    return ratePass(pass1) > ratePass(pass2);
}

bool PassGenerator::passesEqual(AI::Passing::Pass pass1, AI::Passing::Pass pass2)
{
    double max_position_difference_meters =
        Util::DynamicParameters::AI::Passing::pass_equality_max_position_difference_meters
            .value();
    double max_time_difference_seconds =
        Util::DynamicParameters::AI::Passing::
            pass_equality_max_start_time_difference_seconds.value();
    double max_speed_difference =
        Util::DynamicParameters::AI::Passing::
            pass_equality_max_speed_difference_meters_per_second.value();

    double receiver_position_difference =
        (pass1.receiverPoint() - pass2.receiverPoint()).len();
    double passer_position_difference = (pass1.passerPoint() - pass2.passerPoint()).len();
    double time_difference  = (pass1.startTime() - pass2.startTime()).getSeconds();
    double speed_difference = pass1.speed() - pass2.speed();

    return std::abs(receiver_position_difference) < max_position_difference_meters &&
           std::abs(passer_position_difference) < max_position_difference_meters &&
           std::abs(time_difference) < max_time_difference_seconds &&
           std::abs(speed_difference) < max_speed_difference;
}

std::array<double, PassGenerator::NUM_PARAMS_TO_OPTIMIZE>
PassGenerator::convertPassToArray(Pass pass)
{
    return {pass.receiverPoint().x(), pass.receiverPoint().y(), pass.speed(),
            pass.startTime().getSeconds()};
}

Pass PassGenerator::convertArrayToPass(
    std::array<double, PassGenerator::NUM_PARAMS_TO_OPTIMIZE> array)
{
    // Take ownership of the passer_point for the duration of this function
    std::lock_guard<std::mutex> passer_point_lock(passer_point_mutex);

    // Clamp the time to be >= 0, otherwise the TimeStamp will throw an exception
    double clamped_time = std::max(0.0, array.at(3));

    return Pass(passer_point, Point(array.at(0), array.at(1)), array.at(2),
                Timestamp::fromSeconds(clamped_time));
}
