#include "replay_backend.h"

#include <cstdlib>

#include "software/util/generic_factory/generic_factory.h"

ReplayBackend::ReplayBackend(std::shared_ptr<const BackendConfig> config)
    : replay_reader(
          config->getFullSystemMainCommandLineArgs()->getReplayInputDir()->value()),
      pull_from_replay_thread(
          boost::bind(&ReplayBackend::continuouslyPullFromReplayFiles, this))
{
}

void ReplayBackend::onValueReceived(TbotsProto::PrimitiveSet primitives)
{
    // update the time when the backend received the last primitive message. this is
    // used to check when we should exit.
    std::scoped_lock lock(last_primitive_received_time_mutex);
    last_primitive_received_time = std::chrono::steady_clock::now();
}

// do nothing
void ReplayBackend::onValueReceived(World world) {}

void ReplayBackend::continuouslyPullFromReplayFiles()
{
    while (auto sensor_msg_or_null = replay_reader.getNextMsg<SensorProto>())
    {
        auto this_msg_received_time = std::chrono::duration<double>(
            sensor_msg_or_null->backend_received_time().epoch_timestamp_seconds());

        if (last_msg_received_time && last_msg_replayed_time)
        {
            // replicate the timing of messages by sleeping until a time such that
            // the *total* time between the last message sent and the current message sent
            // is equal to the duration between the last message's backend_received_time
            // and the current message's backend_received_time
            auto time_between_last_and_cur_msg =
                (this_msg_received_time - *last_msg_received_time);
            if (time_between_last_and_cur_msg > std::chrono::duration<double>(0))
            {
                std::this_thread::sleep_until(
                    *last_msg_replayed_time +
                    (this_msg_received_time - *last_msg_received_time));
            }
        }
        this->sendValueToObservers(*sensor_msg_or_null);
        last_msg_replayed_time = std::chrono::steady_clock::now();
        last_msg_received_time = this_msg_received_time;
    }

    bool exit = false;
    while (!exit)
    {
        // wait until it has been LAST_PRIMITIVE_TO_SHUTDOWN_DURATION since the last
        // primitive was received by the backend to shutdown, to allow for downstream
        // components to fully finish processing data
        {
            // create another scope in which to hold the lock
            std::scoped_lock lock(last_primitive_received_time_mutex);
            if (!last_primitive_received_time ||
                std::chrono::steady_clock::now() - *last_primitive_received_time >=
                    LAST_PRIMITIVE_TO_SHUTDOWN_DURATION)
            {
                LOG(INFO) << "Reached end of replay, exiting";
                exit = true;
            }
        }
        // wait for CHECK_LAST_PRIMITIVE_TIME_DURATION before checking the above
        // condition again
        std::this_thread::sleep_for(CHECK_LAST_PRIMITIVE_TIME_DURATION);
    }
    std::exit(0);
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, ReplayBackend, BackendConfig> factory;
