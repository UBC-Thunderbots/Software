#include "software/ai/passing/pass_generator.h"

#include <algorithm>
#include <numeric>

#include "software/ai/passing/cost_function.h"
#include "software/ai/passing/pass_generator.h"

PassGenerator::PassGenerator(const World& world, const Point& passer_point,
                             const PassType& pass_type, bool running_deterministically)
    : running_deterministically(running_deterministically),
      updated_world(world),
      world(world),
      passer_robot_id(std::nullopt),
      optimizer(optimizer_param_weights),
      passer_point(passer_point),
      best_known_pass({0, 0}, {0, 0}, 0, Timestamp::fromSeconds(0)),
      target_region(std::nullopt),
      // We initialize the random number generator with a specific value to
      // allow generated passes to be deterministic. The value used here has
      // no special meaning.
      random_num_gen(13),
      pass_type(pass_type),
      in_destructor(false)
{
    // Generate the initial set of passes
    passes_to_optimize = generatePasses(getNumPassesToOptimize());

    // Start the thread to do the pass generation in the background
    // The lambda expression here is needed so that we can call
    // `continuouslyGeneratePasses()`, which is not a static function
    if (!running_deterministically)
    {
        pass_generation_thread =
            std::thread([this]() { return continuouslyGeneratePasses(); });
    }
}

void PassGenerator::setWorld(World world)
{
    // Take ownership of the updated world for the duration of this function
    std::lock_guard<std::mutex> updated_world_lock(updated_world_mutex);

    // Update the world
    this->updated_world = std::move(world);
}

void PassGenerator::setPasserPoint(Point passer_point)
{
    // Take ownership of the passer_point for the duration of this function
    std::lock_guard<std::mutex> passer_point_lock(passer_point_mutex);

    // Update the passer point
    this->passer_point = passer_point;
}

void PassGenerator::setPasserRobotId(unsigned int robot_id)
{
    // Take ownershp of the passer robot id for the duration of this function
    std::lock_guard<std::mutex> passer_robot_id_lock(passer_robot_id_mutex);

    this->passer_robot_id = robot_id;
}

PassWithRating PassGenerator::getBestPassSoFar()
{
    // If we're running deterministically, then we need to manually optimize the
    // passes rather then assuming the optimization thread has done the work for us
    if (running_deterministically)
    {
        for (size_t i = 0; i < NUM_ITERS_PER_DETERMINISTIC_CALL; i++)
        {
            updateAndOptimizeAndPrunePasses();
        }
    }

    // Take ownership of the best_known_pass for the rest of this function
    std::lock_guard<std::mutex> best_known_pass_lock(best_known_pass_mutex);

    Pass best_known_pass_copy = best_known_pass;
    return PassWithRating{std::move(best_known_pass_copy), ratePass(best_known_pass)};
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
    if (!running_deterministically)
    {
        pass_generation_thread.join();
    }
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

        updateAndOptimizeAndPrunePasses();

        // Yield to allow other threads to run. This is particularly important if we
        // have this thread and another running on one core
        std::this_thread::yield();

        // Take ownership of the `in_destructor` flag so we can use it for the conditional
        // check
        in_destructor_mutex.lock();
    }
}

void PassGenerator::updateAndOptimizeAndPrunePasses()
{
    // Copy over the updated world and remove the passer robot
    world_mutex.lock();
    updated_world_mutex.lock();

    world = updated_world;

    // Update the passer point for all the passes
    updated_world_mutex.unlock();
    world_mutex.unlock();

    passer_point_mutex.lock();
    updatePasserPointOfAllPasses(passer_point);
    passer_point_mutex.unlock();
    optimizePasses();
    pruneAndReplacePasses();
    saveBestPass();
}

void PassGenerator::optimizePasses()
{
    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    const auto objective_function =
        [this](const std::array<double, NUM_PARAMS_TO_OPTIMIZE>& pass_array) {
            try
            {
                Pass pass = convertArrayToPass(pass_array);
                return ratePass(pass);
            }
            catch (std::invalid_argument& e)
            {
                return 0.0;
            }
        };

    // Run gradient descent to optimize the passes to for the requested number
    // of iterations
    // NOTE: Parallelizing this `for` loop would probably be safe and potentially more
    //       performant
    std::vector<Pass> updated_passes;
    for (Pass& pass : passes_to_optimize)
    {
        auto pass_array = optimizer.maximize(objective_function, convertPassToArray(pass),
                                             DynamicParameters->getAIConfig()
                                                 ->getPassingConfig()
                                                 ->NumberOfGradientDescentStepsPerIter()
                                                 ->value());
        try
        {
            updated_passes.emplace_back(convertArrayToPass(pass_array));
        }
        catch (std::invalid_argument& e)
        {
            // Sometimes the gradient descent algorithm could return an invalid pass, if
            // so, we can just ignore it and carry on
        }
    }
    passes_to_optimize = updated_passes;
}

void PassGenerator::pruneAndReplacePasses()
{
    // Sort the passes by decreasing quality
    std::sort(passes_to_optimize.begin(), passes_to_optimize.end(),
              [this](Pass p1, Pass p2) { return comparePassQuality(p1, p2); });

    // Merge Passes That Are Similar
    // We start by assuming that the most similar passes will be right beside each other,
    // then iterate over the entire list, building a new list as we go by only adding
    // elements when they are dissimilar enough from the last element we added
    // NOTE: This flips the passes so they are sorted by increasing quality
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

    // Replace the least promising passes
    passes_to_optimize.erase(
        passes_to_optimize.begin() + getNumPassesToKeepAfterPruning(),
        passes_to_optimize.end());

    // Generate new passes to replace the ones we just removed
    int num_new_passes =
        getNumPassesToOptimize() - static_cast<int>(passes_to_optimize.size());
    if (num_new_passes > 0)
    {
        std::vector<Pass> new_passes = generatePasses(num_new_passes);
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
    if (passes_to_optimize.empty())
    {
        throw std::runtime_error(
            "passes_to_optimize is empty in PassGenerator, this should never happen");
    }
    best_known_pass = passes_to_optimize[0];
}

unsigned int PassGenerator::getNumPassesToKeepAfterPruning()
{
    // We want to use the parameter value for this, but clamp it so that it is
    // <= the number of passes we're optimizing
    return std::min(static_cast<unsigned int>(DynamicParameters->getAIConfig()
                                                  ->getPassingConfig()
                                                  ->NumPassesToKeepAfterPruning()
                                                  ->value()),
                    getNumPassesToOptimize());
}

unsigned int PassGenerator::getNumPassesToOptimize()
{
    // We want to use the parameter value for this, but clamp it so that it is
    // >= 1 so we are always optimizing at least one pass
    return std::max(static_cast<unsigned int>(DynamicParameters->getAIConfig()
                                                  ->getPassingConfig()
                                                  ->NumPassesToOptimize()
                                                  ->value()),
                    static_cast<unsigned int>(1));
}

void PassGenerator::updatePasserPointOfAllPasses(const Point& new_passer_point)
{
    for (Pass& pass : passes_to_optimize)
    {
        pass =
            Pass(new_passer_point, pass.receiverPoint(), pass.speed(), pass.startTime());
    }
}

double PassGenerator::ratePass(const Pass& pass)
{
    // Take ownership of world, target_region, passer_robot_id for the duration of this
    // function
    std::lock_guard<std::mutex> world_lock(world_mutex);
    std::lock_guard<std::mutex> target_region_lock(target_region_mutex);
    std::lock_guard<std::mutex> passer_robot_id_lock(passer_robot_id_mutex);

    double rating = 0;
    try
    {
        rating = ::ratePass(world, pass, target_region, passer_robot_id, pass_type);
    }
    catch (std::invalid_argument& e)
    {
        // If the pass is invalid, just rate it as poorly as possible
        rating = 0;
    }

    return rating;
}

std::vector<Pass> PassGenerator::generatePasses(unsigned long num_passes_to_gen)
{
    // Take ownership of world for the duration of this function
    std::lock_guard<std::mutex> world_lock(world_mutex);

    std::uniform_real_distribution x_distribution(-world.field().xLength() / 2,
                                                  world.field().xLength() / 2);
    std::uniform_real_distribution y_distribution(-world.field().yLength() / 2,
                                                  world.field().yLength() / 2);

    double curr_time             = world.getMostRecentTimestamp().toSeconds();
    double min_start_time_offset = DynamicParameters->getAIConfig()
                                       ->getPassingConfig()
                                       ->MinTimeOffsetForPassSeconds()
                                       ->value();
    double max_start_time_offset = DynamicParameters->getAIConfig()
                                       ->getPassingConfig()
                                       ->MaxTimeOffsetForPassSeconds()
                                       ->value();
    std::uniform_real_distribution start_time_distribution(
        curr_time + min_start_time_offset, curr_time + max_start_time_offset);
    std::uniform_real_distribution speed_distribution(DynamicParameters->getAIConfig()
                                                          ->getPassingConfig()
                                                          ->MinPassSpeedMPerS()
                                                          ->value(),
                                                      DynamicParameters->getAIConfig()
                                                          ->getPassingConfig()
                                                          ->MaxPassSpeedMPerS()
                                                          ->value());

    std::vector<Pass> passes;
    for (unsigned i = 0; i < num_passes_to_gen; i++)
    {
        Point receiver_point(x_distribution(random_num_gen),
                             y_distribution(random_num_gen));
        Timestamp start_time =
            Timestamp::fromSeconds(start_time_distribution(random_num_gen));
        double pass_speed = speed_distribution(random_num_gen);

        Pass p(passer_point, receiver_point, pass_speed, start_time);
        passes.emplace_back(p);
    }

    return passes;
}

bool PassGenerator::comparePassQuality(const Pass& pass1, const Pass& pass2)
{
    return ratePass(pass1) > ratePass(pass2);
}

bool PassGenerator::passesEqual(Pass pass1, Pass pass2)
{
    double max_position_difference_meters =
        DynamicParameters->getAIConfig()
            ->getPassingConfig()
            ->PassEqualityMaxPositionDifferenceMeters()
            ->value();
    double max_time_difference_seconds = DynamicParameters->getAIConfig()
                                             ->getPassingConfig()
                                             ->PassEqualityMaxStartTimeDifferenceSeconds()
                                             ->value();
    double max_speed_difference = DynamicParameters->getAIConfig()
                                      ->getPassingConfig()
                                      ->PassEqualityMaxSpeedDifferenceMetersPerSecond()
                                      ->value();

    double receiver_position_difference =
        (pass1.receiverPoint() - pass2.receiverPoint()).length();
    double passer_position_difference =
        (pass1.passerPoint() - pass2.passerPoint()).length();
    double time_difference  = (pass1.startTime() - pass2.startTime()).toSeconds();
    double speed_difference = pass1.speed() - pass2.speed();

    return std::abs(receiver_position_difference) < max_position_difference_meters &&
           std::abs(passer_position_difference) < max_position_difference_meters &&
           std::abs(time_difference) < max_time_difference_seconds &&
           std::abs(speed_difference) < max_speed_difference;
}

std::array<double, PassGenerator::NUM_PARAMS_TO_OPTIMIZE>
PassGenerator::convertPassToArray(const Pass& pass)
{
    // Take ownership of the world for the duration of this function
    std::lock_guard<std::mutex> world_lock(world_mutex);

    return {pass.receiverPoint().x(), pass.receiverPoint().y(), pass.speed(),
            pass.startTime().toSeconds()};
}

Pass PassGenerator::convertArrayToPass(
    const std::array<double, PassGenerator::NUM_PARAMS_TO_OPTIMIZE>& array)
{
    // Take ownership of the passer_point and world for the duration of this function
    std::lock_guard<std::mutex> passer_point_lock(passer_point_mutex);
    std::lock_guard<std::mutex> world_lock(world_mutex);

    // Clamp the time to be >= 0, otherwise the TimeStamp will throw an exception
    double time_offset_seconds = std::max(0.0, array.at(3));

    return Pass(passer_point, Point(array.at(0), array.at(1)), array.at(2),
                Timestamp::fromSeconds(time_offset_seconds));
}
