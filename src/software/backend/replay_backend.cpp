#include "replay_backend.h"
#include "software/util/design_patterns/generic_factory.h"

const std::string ReplayBackend::name = "replay";

ReplayBackend::ReplayBackend(
    std::shared_ptr<const ReplayBackendConfig> replay_backend_config)
    :Backend(), replay_reader(replay_backend_config->ReplayPath()->value()),
     pull_from_replay_thread(boost::bind(&ReplayBackend::continuouslyPullFromReplayFiles, this))
{}

// do nothing
// TODO: perhaps we want to encode the output primitives to proto and log them?
void ReplayBackend::onValueReceived(ConstPrimitiveVectorPtr primitives) {}

// do nothing
void ReplayBackend::onValueReceived(World world) {}

void ReplayBackend::continuouslyPullFromReplayFiles() {
    while (auto sensor_msg_or_null = replay_reader.getNextMsg()) {
        this->sendValueToObservers(*sensor_msg_or_null);
        if (sensor_msg_or_null->has_ssl_vision_msg()) {
            LOG(INFO) << "Replayed vision frame with t_sent=="
                      << sensor_msg_or_null->ssl_vision_msg().detection().t_sent();
        }
        // TODO: timestamps in sensor_msg!
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    throw std::out_of_range("Reached end of replay!");
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, ReplayBackend> factory;