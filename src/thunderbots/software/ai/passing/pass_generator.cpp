#include <algorithm>
#include <numeric>
#include "ai/passing/pass_generator.h"
#include "pass_generator.h"
#include "ai/passing/evaluation.h"


using namespace AI::Passing;


PassGenerator::PassGenerator(double min_reasonable_pass_quality):
min_reasonable_pass_quality(min_reasonable_pass_quality),
optimizer(optimizer_param_weights),
best_known_pass(std::nullopt),
in_destructor(false)
{
    // Generate the initial set of passes
    passes_to_optimize = generatePasses(this->num_passes_to_optimize);

    // Start the thread to do the pass generation in the background
    // The lambda expression here is needed so that we can call
    // `continuouslyGeneratePasses()`, which is not a static function
    pass_generation_thread = std::thread([this](){ return continuouslyGeneratePasses(); });
}

void PassGenerator::setWorld(World world) {
    // Take ownership of the world for the duration of this function
    std::lock_guard<std::mutex> world_lock(world_mutex);

    // Update the world
    this->world = std::move(world);
}

void PassGenerator::setPasserPoint(Point passer_point) {
    // Take ownership of the passer_point for the duration of this function
    std::lock_guard<std::mutex> passer_point_lock(passer_point_mutex);

    // Update the passer point
    this->passer_point = passer_point;
}

std::optional<Pass> PassGenerator::getBestPassSoFar() {
    // Take ownership of the best_known_pass for the duration of this function
    std::lock_guard<std::mutex> best_known_pass_lock(best_known_pass_mutex);

    return best_known_pass;
}

PassGenerator::~PassGenerator() {
    // Set this flag so pass_generation_thread knows to end (also making sure to
    // properly take and give ownership of the flag)
    in_destructor_mutex.lock();
    in_destructor = true;
    in_destructor_mutex.unlock();

    // Join to pass_generation_thread so that we wait for it to exit before destructing
    // the thread object. If we do not wait for thread to finish executing, it will
    // call `std::terminate` and kill our whole program when it finishes
    pass_generation_thread.join();
}

void PassGenerator::continuouslyGeneratePasses() {
    // Take ownership of the in_destructor flag so we can use it for the conditional
    // check
    in_destructor_mutex.lock();
    while(!in_destructor){
        // Give up ownership of the in_destructor flag now that we're done the
        // conditional check
        in_destructor_mutex.unlock();

        optimizePasses();
        pruneAndReplacePasses();
        saveBestPath();

        // Yield to allow other threads to run. This is particularly important if we
        // have this thread and another running on one core
        std::this_thread::yield();

        // Take ownership of the `in_destructor` flag so we can use it for the conditional
        // check
        in_destructor_mutex.lock();
    }
}

void PassGenerator::optimizePasses() {
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
}

void PassGenerator::pruneAndReplacePasses() {
    // Sort the passes by quality
    std::sort(passes_to_optimize.begin(), passes_to_optimize.end(),
              [this](Pass p1, Pass p2) { return comparePassQuality(p1, p2); }
    );

    // TODO: clean this up
    // Replace the least promising passes with newly generated passes
    if (num_passes_to_keep_after_pruning < num_passes_to_optimize){
        std::vector<Pass> new_passes = generatePasses(num_passes_to_optimize -
                                                      num_passes_to_keep_after_pruning);
        if (num_passes_to_keep_after_pruning < passes_to_optimize.size()) {
            passes_to_optimize.erase(passes_to_optimize.begin() + num_passes_to_keep_after_pruning,
                    passes_to_optimize.end());
        }
        passes_to_optimize.insert(passes_to_optimize.begin() + num_passes_to_keep_after_pruning, new_passes.begin(), new_passes.end());
    }

}

void PassGenerator::saveBestPath() {
    // Take ownership of the best_known_pass for the duration of this function
    std::lock_guard<std::mutex> best_known_pass_lock(best_known_pass_mutex);

    std::sort(passes_to_optimize.begin(), passes_to_optimize.end(),
              [this](auto pass1, auto pass2) { return comparePassQuality(pass1, pass2);});
    if (!passes_to_optimize.empty() && ratePass(passes_to_optimize[0]) > min_reasonable_pass_quality){
        best_known_pass = std::optional(passes_to_optimize[0]);
    } else {
        best_known_pass = std::nullopt;
    }
}

double PassGenerator::ratePass(Pass pass) {
    // Take ownership of world for the duration of this function
    std::lock_guard<std::mutex> world_lock(world_mutex);

    double pass_quality = getStaticPositionQuality(world.field(), pass.receiverPoint());

    double distance_to_goal = Vector(pass.receiverPoint().x() - world.field().enemyGoal().x(),
            pass.receiverPoint().y() - world.field().enemyGoal().y()).len();

    pass_quality = 1 / (1 + std::exp(distance_to_goal - 2.1));

    // TODO: the rest of this function; see the old code

    return pass_quality;
}

std::vector<Pass> PassGenerator::generatePasses(unsigned long num_paths_to_gen) {
    // Take ownership of world for the duration of this function
    std::lock_guard<std::mutex> world_lock(world_mutex);

    std::vector<Pass> v;

    Pass p(Point(0,0), Point(0,0), 0, Timestamp::fromSeconds(0));
    for (int i = 0; i < num_paths_to_gen; i++){
        v.emplace_back(p);
    }

    // TODO: implement this function properly; see the old code

    return v;
}

bool PassGenerator::comparePassQuality(const Pass& pass1, const Pass& pass2) {
    return ratePass(pass1) > ratePass(pass2);
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
    // Take ownership of the passer_point for the duration of this function
    std::lock_guard<std::mutex> passer_point_lock(passer_point_mutex);

    // TODO: Set the timestamp properly here, just setting to 0 to avoid negative values......
    return Pass(passer_point, Point(array.at(0), array.at(1)), array.at(2),
            Timestamp::fromSeconds(0));
}
