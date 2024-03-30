#include "software/ai/hl/stp/strategy/pass_strategy.h"

#include "software/ai/passing/eighteen_zone_pitch_division.h"

PassStrategy::PassStrategy(const TbotsProto::PassingConfig& passing_config,
                           const Field& field)
    : world_ptr_(nullptr),
      pass_generator_(std::make_shared<const EighteenZonePitchDivision>(field),
                      passing_config),
      passing_thread_(&PassStrategy::evaluatePassOptions, this),
      latest_pass_eval_(nullptr),
      end_analysis_(false)
{
}

PassStrategy::~PassStrategy()
{
    end_analysis_ = true;

    world_available_cv_.notify_one();

    passing_thread_.join();
}

std::shared_ptr<PassEvaluation<EighteenZoneId>> PassStrategy::getPassEvaluation()
{
    std::unique_lock<std::mutex> lock(pass_evaluation_lock_);
    pass_available_cv_.wait(lock, [&] { return latest_pass_eval_ != nullptr; });

    return latest_pass_eval_;
}

void PassStrategy::evaluatePassOptions()
{
    {
        LOG(DEBUG) << "PassStrategy: waiting for World Lock";
        std::unique_lock<std::mutex> lock(world_lock_);

        world_available_cv_.wait(lock,
                                 [&] { return world_ptr_ != nullptr || end_analysis_; });
    }

    while (!end_analysis_)
    {
        std::shared_ptr<PassEvaluation<EighteenZoneId>> pass_eval;
        WorldPtr world_ptr;
        {
            const std::lock_guard<std::mutex> lock(world_lock_);
            world_ptr = world_ptr_;
        }

        pass_eval = std::make_shared<PassEvaluation<EighteenZoneId>>(
            pass_generator_.generatePassEvaluation(*world_ptr));

        {
            const std::lock_guard<std::mutex> lock(pass_evaluation_lock_);
            latest_pass_eval_ = pass_eval;
            pass_available_cv_.notify_one();
        }
    }
}

void PassStrategy::updateWorld(const WorldPtr& world_ptr)
{
    const std::lock_guard<std::mutex> lock(world_lock_);
    world_ptr_ = world_ptr;
    world_available_cv_.notify_one();
}
