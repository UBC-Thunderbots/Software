#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/multithreading/subject.h"
#include "software/replay/replay_logger.h"
#include "software/replay/replay_reader.h"

constexpr const char* REPLAY_TEST_PATH_SUFFIX = "software/replay/test_replay";

namespace fs = std::experimental::filesystem;

// hahaha test subject because it's a subject but also a test subject
class TestSubject : public Subject<SensorMsg>
{
   public:
    void sendValue(SensorMsg val)
    {
        sendValueToObservers(val);
    }
};

TEST(ReplayTest, test_read_and_write_replay)
{
    // unfortunately due to the unavailablity of ordering tests and functions that
    // generate test files that actually work, we have to read recorded replays back,
    // write them, and then read them again in the same test
    // TODO: record replay data with refbox running

    std::vector<SensorMsg> read_replay_frames;

    ReplayReader reader(fs::current_path() / REPLAY_TEST_PATH_SUFFIX);
    while (reader.hasNextFrame()) {
        auto frame = reader.getNextFrame();
        read_replay_frames.emplace_back(frame);
    }
    // do a quick sanity check here to assert that the timestamps are monotonically increasing
    double max_timestamp = 0.f;
    for (const auto& msg : read_replay_frames) {
        auto ts = msg.ssl_vision_msg().detection().t_sent();
        if (ts < max_timestamp) {
            FAIL() << "t_sent " << ts << " should have appeared after " << max_timestamp;
        } else {
            max_timestamp = ts;
        }
    }

    // write the read frames to another replay directory
    auto output_path = fs::current_path() / "replaytest";
    std::shared_ptr<Observer<SensorMsg>> logger_ptr =
            std::make_shared<ReplayLogger>(output_path);
    TestSubject subject;
    subject.registerObserver(logger_ptr);
    for (const auto msg : read_replay_frames) {
        subject.sendValue(msg);
        // we have 12000 frames of replay so we have to try to not fill the buffer
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // unfortunately we have to do this
    std::this_thread::sleep_for(std::chrono::seconds(5));
    for (const auto& dir_entry : fs::directory_iterator(output_path)) {
        std::cout << dir_entry << std::endl;
    }

    // compare against the messages that we read
    std::vector<SensorMsg> written_read_replay_frames;
    ReplayReader reader2(output_path);
    while (reader2.hasNextFrame()) {
        written_read_replay_frames.emplace_back(reader2.getNextFrame());
    }

    EXPECT_EQ(written_read_replay_frames.size(), read_replay_frames.size())
        << "Replay written and read back does not have the same number of frames!";

    for (size_t i = 0; i < written_read_replay_frames.size(); i++) {
        bool eq = google::protobuf::util::MessageDifferencer::ApproximatelyEquivalent(
                read_replay_frames[i], written_read_replay_frames[i]
                );
        EXPECT_TRUE(eq);
    }
}