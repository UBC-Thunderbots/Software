#pragma once

#include <condition_variable>
#include <thread>

#include "software/ai/passing/pass_generator.hpp"

/**
 * This class wraps a `BasePassGenerator` and uses it to generate passes on
 * a separate thread.
 *
 * @tparam ZoneEnum the enum used to identify pitch division zones
 */
template <class ZoneEnum>
class ThreadedPassGenerator
{
   public:
    /**
     * Creates a new ThreadedPassGenerator with the given pitch division.
     *
     * The PassGenerator will use this pitch division to guide initial random samples
     * in each zone after the pitch has been divided.
     *
     * @param pitch_division the pitch division to use when looking for passes
     * @param passing_config the passing configuration
     */
    ThreadedPassGenerator(
        std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
        TbotsProto::PassingConfig passing_config);

    ~ThreadedPassGenerator();

    /**
     * Gets the latest generated pass evaluation.
     *
     * @return the best currently known pass evaluation
     */
    std::shared_ptr<PassEvaluation<ZoneEnum>> getPassEvaluation();

    /**
     * Update the World provided to the pass generator to compute pass evaluations on.
     *
     * @param world_ptr the World to provide to the pass generator
     */
    void updateWorld(const WorldPtr& world_ptr);

   private:
    /**
     * Continually generates pass evaluations and updates `latest_pass_eval_`
     * with the latest generated pass evaluation.
     *
     * This function will keep running until the `end_analysis_` flag is true.
     */
    void evaluatePassOptions();

    // Thread that the pass generator will run on
    std::thread pass_generator_thread_;

    // The pass generator used to compute pass evaluations
    PassGenerator<ZoneEnum> pass_generator_;

    // The latest generated pass evaluation
    std::shared_ptr<PassEvaluation<ZoneEnum>> latest_pass_eval_;

    // Synchronization primitives for `latest_pass_eval_`
    std::mutex pass_eval_mutex_;
    std::condition_variable pass_eval_cv_;

    // World provided to the pass generator
    WorldPtr world_ptr_;

    // Synchronization primitives for `world_ptr_`
    std::mutex world_mutex_;
    std::condition_variable world_cv_;

    // Flag that will terminate the pass generator thread when set to true
    std::atomic<bool> end_analysis_;
};

template <class ZoneEnum>
ThreadedPassGenerator<ZoneEnum>::ThreadedPassGenerator(
    std::shared_ptr<const FieldPitchDivision<ZoneEnum>> pitch_division,
    TbotsProto::PassingConfig passing_config)
    : pass_generator_thread_(&ThreadedPassGenerator::evaluatePassOptions, this),
      pass_generator_(pitch_division, passing_config),
      latest_pass_eval_(nullptr),
      world_ptr_(nullptr),
      end_analysis_(false)
{
}

template <class ZoneEnum>
ThreadedPassGenerator<ZoneEnum>::~ThreadedPassGenerator()
{
    end_analysis_ = true;
    world_cv_.notify_one();
    pass_generator_thread_.join();
}

template <class ZoneEnum>
std::shared_ptr<PassEvaluation<ZoneEnum>>
ThreadedPassGenerator<ZoneEnum>::getPassEvaluation()
{
    std::unique_lock<std::mutex> lock(pass_eval_mutex_);
    pass_eval_cv_.wait(lock, [&] { return latest_pass_eval_ != nullptr; });
    return latest_pass_eval_;
}

template <class ZoneEnum>
void ThreadedPassGenerator<ZoneEnum>::evaluatePassOptions()
{
    {
        std::unique_lock<std::mutex> lock(world_mutex_);
        world_cv_.wait(lock, [&] { return world_ptr_ != nullptr || end_analysis_; });
    }

    while (!end_analysis_)
    {
        std::shared_ptr<PassEvaluation<ZoneEnum>> pass_eval;
        WorldPtr world_ptr;
        {
            std::lock_guard<std::mutex> lock(world_mutex_);
            world_ptr = world_ptr_;
        }

        pass_eval = std::make_shared<PassEvaluation<ZoneEnum>>(
            pass_generator_.generatePassEvaluation(*world_ptr));

        {
            std::lock_guard<std::mutex> lock(pass_eval_mutex_);
            latest_pass_eval_ = pass_eval;
            pass_eval_cv_.notify_one();
        }
    }
}

template <class ZoneEnum>
void ThreadedPassGenerator<ZoneEnum>::updateWorld(const WorldPtr& world_ptr)
{
    std::lock_guard<std::mutex> lock(world_mutex_);
    world_ptr_ = world_ptr;
    world_cv_.notify_one();
}
