#include "replay_backend.h"

#include <cstdlib>

#include "software/util/design_patterns/generic_factory.h"

ReplayBackend::ReplayBackend(const std::string& replay_input_dir)
    : Backend(),
      replay_reader(replay_input_dir),
      pull_from_replay_thread(
          boost::bind(&ReplayBackend::continuouslyPullFromReplayFiles, this)),
      last_msg_received_time(std::nullopt),
      last_msg_replayed_time(std::nullopt),
      last_primitive_received_time(std::nullopt),
      last_primitive_received_time_mutex()
{
}

// TODO: https://github.com/UBC-Thunderbots/Software/issues/1745
//       perhaps we want to encode the output primitives to proto and log them?
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

    // wait 1 second until the last primitive is received by the backend
    // to exit
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
static TGenericFactory<std::string, Backend, ReplayBackend> factory;
