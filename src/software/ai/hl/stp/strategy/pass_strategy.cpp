#include "software/ai/hl/stp/strategy/pass_strategy.h"

#include "software/ai/passing/eighteen_zone_pitch_division.h"

PassStrategy::PassStrategy(const TbotsProto::PassingConfig& passing_config,
                           const Field& field)
    : pass_generator_(std::make_shared<const EighteenZonePitchDivision>(field),
                      passing_config),
      passing_thread_(&PassStrategy::evaluatePassOptions, this),
      latest_pass_eval_(nullptr),
      end_analysis_(false)
{
}

PassStrategy::~PassStrategy()
{
    end_analysis_ = true;
}

PassEvaluation<EighteenZoneId> PassStrategy::getPassEvaluation()
{
    std::unique_lock<std::mutex> lock(pass_evaluation_lock_);
    pass_available_cv_.wait(lock, [&] { return latest_pass_eval_ != nullptr; });

    return *latest_pass_eval_;
}

void PassStrategy::evaluatePassOptions()
{
    {
        std::unique_lock<std::mutex> lock(world_lock_);

        world_available_cv_.wait(lock, [&] { return current_world_.has_value(); });
    }

    while (!end_analysis_)
    {
        std::unique_ptr<PassEvaluation<EighteenZoneId>> pass_eval;
        {
            const std::lock_guard<std::mutex> lock(world_lock_);
            pass_eval = std::make_unique<PassEvaluation<EighteenZoneId>>(
                pass_generator_.generatePassEvaluation(current_world_.value()));
        }

        {
            const std::lock_guard<std::mutex> lock(pass_evaluation_lock_);
            latest_pass_eval_ = std::move(pass_eval);
            pass_available_cv_.notify_one();
        }
    }
}

void PassStrategy::updateWorld(const World& world)
{
    const std::lock_guard<std::mutex> lock(world_lock_);
    current_world_.emplace(world);
    world_available_cv_.notify_one();
}
