#include "software/ai/hl/stp/strategy/pass_strategy.h"

#include "software/ai/passing/eighteen_zone_pitch_division.h"

PassStrategy::PassStrategy(const TbotsProto::PassingConfig& passing_config,
                           const Field& field)
    : current_world_(std::nullopt),
      pass_generator_(std::make_shared<const EighteenZonePitchDivision>(field),
                      passing_config),
      passing_thread_(&PassStrategy::evaluatePassOptions, this),
      latest_pass_eval_(nullptr),
      end_analysis_(false)
{
    LOG(DEBUG) << "PassStrategy starting...";
}

PassStrategy::~PassStrategy()
{
    LOG(DEBUG) << "PassStrategy exiting...";
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

        world_available_cv_.wait(
            lock, [&] { return current_world_.has_value() || end_analysis_; });
    }

    while (!end_analysis_)
    {
        std::shared_ptr<PassEvaluation<EighteenZoneId>> pass_eval;
        std::unique_ptr<const World> world;
        {
            const std::lock_guard<std::mutex> lock(world_lock_);

            const World& copy_world = current_world_.value();
            world = std::unique_ptr<const World>(new World(copy_world));
        }

        pass_eval = std::make_shared<PassEvaluation<EighteenZoneId>>(
            pass_generator_.generatePassEvaluation(*world));

        {
            const std::lock_guard<std::mutex> lock(pass_evaluation_lock_);
            latest_pass_eval_ = pass_eval;
            pass_available_cv_.notify_one();
        }
    }
}

void PassStrategy::updateWorld(const World& world)
{
    LOG(DEBUG) << "PassStrategy updating the World";
    const std::lock_guard<std::mutex> lock(world_lock_);
    current_world_.emplace(world);
    world_available_cv_.notify_one();
}
