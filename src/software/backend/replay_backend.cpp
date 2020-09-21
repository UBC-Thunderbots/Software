#include "replay_backend.h"

#include "software/util/design_patterns/generic_factory.h"

const std::string ReplayBackend::name = "replay";

ReplayBackend::ReplayBackend(
    std::shared_ptr<const ReplayBackendConfig> replay_backend_config)
    : Backend(),
      replay_reader(replay_backend_config->ReplayPath()->value()),
      pull_from_replay_thread(
          boost::bind(&ReplayBackend::continuouslyPullFromReplayFiles, this)),
      last_msg_received_time(std::nullopt),
      last_msg_replayed_time(std::nullopt)
{
}

// do nothing
// TODO: perhaps we want to encode the output primitives to proto and log them?
void ReplayBackend::onValueReceived(TbotsProto::PrimitiveSet primitives) {}

// do nothing
void ReplayBackend::onValueReceived(World world) {}

void ReplayBackend::continuouslyPullFromReplayFiles()
{
    while (auto sensor_msg_or_null = replay_reader.getNextMsg())
    {
        auto this_msg_received_time = std::chrono::duration<double>(
            sensor_msg_or_null->time_received().epoch_timestamp_seconds());

        if (last_msg_received_time && last_msg_replayed_time)
        {
            std::this_thread::sleep_until(
                *last_msg_replayed_time +
                (this_msg_received_time - *last_msg_received_time));
        }
        this->sendValueToObservers(*sensor_msg_or_null);
        last_msg_replayed_time = std::chrono::steady_clock::now();
        last_msg_received_time = this_msg_received_time;
    }
    throw std::out_of_range("Reached end of replay!");
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, ReplayBackend> factory;