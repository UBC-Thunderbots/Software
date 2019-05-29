#include "ai/passing/pass_generator.h"

#include <algorithm>
#include <numeric>

#include "ai/passing/evaluation.h"
#include "pass_generator.h"
#include "util/canvas_messenger/canvas_messenger.h"

using namespace AI::Passing;
using namespace Util::DynamicParameters::AI::Passing;

PassGenerator::PassGenerator(const World& world, const Point& passer_point)
    : updated_world(world),
      world(world),
      passer_robot_id(std::nullopt),
      optimizer(optimizer_param_weights),
      passer_point(passer_point),
      best_known_pass({0, 0}, {0, 0}, 0, Timestamp::fromSeconds(0)),
      target_region(std::nullopt),
      random_num_gen(random_device()),
      in_destructor(false)
{
    // Generate the initial set of passes
    passes_to_optimize = generatePasses(getNumPassesToOptimize());

    // Start the thread to do the pass generation in the background
    // The lambda expression here is needed so that we can call
    // `continuouslyGeneratePasses()`, which is not a static function
    pass_generation_thread =
        std::thread([this]() { return continuouslyGeneratePasses(); });
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

std::pair<Pass, double> PassGenerator::getBestPassSoFar()
{
    // Take ownership of the best_known_pass for the duration of this function
    std::lock_guard<std::mutex> best_known_pass_lock(best_known_pass_mutex);

    Pass best_known_pass_copy = best_known_pass;
    return std::make_pair<Pass, double>(std::move(best_known_pass_copy),
                                        ratePass(best_known_pass));
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

        // Copy over the updated world and remove the passer robot
        world_mutex.lock();
        updated_world_mutex.lock();
        passer_robot_id_mutex.lock();

        world = updated_world;
        if (passer_robot_id)
        {
            world.mutableFriendlyTeam().removeRobotWithId(*passer_robot_id);
        }

        passer_robot_id_mutex.unlock();
        updated_world_mutex.unlock();
        world_mutex.unlock();

        passer_point_mutex.lock();
        updatePasserPointOfAllPasses(passer_point);
        passer_point_mutex.unlock();
        optimizePasses();
        pruneAndReplacePasses();
        saveBestPass();
        visualizePassesAndPassQualityGradient();

        // Yield to allow other threads to run. This is particularly important if we
        // have this thread and another running on one core
        std::this_thread::yield();

        // Take ownership of the `in_destructor` flag so we can use it for the conditional
        // check
        in_destructor_mutex.lock();
    }
}

void PassGenerator::visualizePassesAndPassQualityGradient()
{
    // Take ownership of the passer point for the duration of this function
    std::lock_guard<std::mutex> passer_point_lock(passer_point_mutex);

    // Draw all the points we have so far
    auto canvas_messenger = Util::CanvasMessenger::getInstance();

    // Get field characteristics and the current time
    world_mutex.lock();
    Rectangle field_area(world.field().enemyCornerNeg(),
                         world.field().friendlyCornerPos());
    double field_length = world.field().length();
    double field_width  = world.field().width();
    world_mutex.unlock();

    auto best_pass_and_score      = getBestPassSoFar();
    auto [best_pass, best_score]  = best_pass_and_score;
    const auto objective_function = [&](Point p) {
        try
        {
            Pass pass(passer_point, p, best_pass.speed(), best_pass.startTime());
            return ratePass(pass);
        }
        catch (std::invalid_argument& e)
        {
            return 0.0;
        }
    };
    canvas_messenger->drawGradient(Util::CanvasMessenger::Layer::PASS_GENERATION,
                                   objective_function, field_area, 0, 1, {0, 0, 255, 160},
                                   {255, 0, 0, 160}, 4);
    canvas_messenger->drawPoint(Util::CanvasMessenger::Layer::PASS_GENERATION,
                                best_pass.receiverPoint(), 0.05, {0, 255, 0, 255});
    for (const Pass& pass : passes_to_optimize)
    {
        canvas_messenger->drawPoint(Util::CanvasMessenger::Layer::PASS_GENERATION,
                                    pass.receiverPoint(), 0.03, {0, 255, 0, 150});
    }

    canvas_messenger->publishAndClearLayer(Util::CanvasMessenger::Layer::PASS_GENERATION);
}

void PassGenerator::optimizePasses()
{
    // The objective function we minimize in gradient descent to improve each pass
    // that we're optimizing
    const auto objective_function =
        [this](std::array<double, NUM_PARAMS_TO_OPTIMIZE> pass_array) {
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
        auto pass_array =
            optimizer.maximize(objective_function, convertPassToArray(pass),
                               number_of_gradient_descent_steps_per_iter.value());
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
    int num_new_passes = getNumPassesToOptimize() - passes_to_optimize.size();
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
    return std::min(static_cast<unsigned int>(num_passes_to_keep_after_pruning.value()),
                    getNumPassesToOptimize());
}

unsigned int PassGenerator::getNumPassesToOptimize()
{
    // We want to use the parameter value for this, but clamp it so that it is
    // >= 1 so we are always optimizing at least one pass
    return std::max(static_cast<unsigned int>(num_passes_to_optimize.value()),
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

double PassGenerator::ratePass(Pass pass)
{
    // Take ownership of world, target_region for the duration of this function
    std::lock_guard<std::mutex> world_lock(world_mutex);
    std::lock_guard<std::mutex> target_region_lock(target_region_mutex);

    double rating = 0;
    try
    {
        rating = ::ratePass(world, pass, target_region);
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

    std::uniform_real_distribution x_distribution(-world.field().length() / 2,
                                                  world.field().length() / 2);
    std::uniform_real_distribution y_distribution(-world.field().width() / 2,
                                                  world.field().width() / 2);
    // TODO (Issue #423): We should use the timestamp from the world instead of the ball
    double curr_time = world.ball().lastUpdateTimestamp().getSeconds();
    double min_start_time_offset =
        Util::DynamicParameters::AI::Passing::min_time_offset_for_pass_seconds.value();
    double max_start_time_offset =
        Util::DynamicParameters::AI::Passing::max_time_offset_for_pass_seconds.value();
    std::uniform_real_distribution start_time_distribution(
        curr_time + min_start_time_offset, curr_time + max_start_time_offset);
    std::uniform_real_distribution speed_distribution(
        Util::DynamicParameters::AI::Passing::min_pass_speed_m_per_s.value(),
        Util::DynamicParameters::AI::Passing::max_pass_speed_m_per_s.value());

    std::vector<Pass> passes;
    for (int i = 0; i < num_passes_to_gen; i++)
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
    // Take ownership of the world for the duration of this function
    std::lock_guard<std::mutex> world_lock(world_mutex);

    return {pass.receiverPoint().x(), pass.receiverPoint().y(), pass.speed(),
            pass.startTime().getSeconds()};
}

Pass PassGenerator::convertArrayToPass(
    std::array<double, PassGenerator::NUM_PARAMS_TO_OPTIMIZE> array)
{
    // Take ownership of the passer_point and world for the duration of this function
    std::lock_guard<std::mutex> passer_point_lock(passer_point_mutex);
    std::lock_guard<std::mutex> world_lock(world_mutex);

    // Clamp the time to be >= 0, otherwise the TimeStamp will throw an exception
    double time_offset_seconds = std::max(0.0, array.at(3));

    return Pass(passer_point, Point(array.at(0), array.at(1)), array.at(2),
                Timestamp::fromSeconds(time_offset_seconds));
}
