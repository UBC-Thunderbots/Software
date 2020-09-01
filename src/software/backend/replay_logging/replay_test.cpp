#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/backend/replay_logging/replay_logger.h"
#include "software/backend/replay_logging/replay_reader.h"
#include "software/multithreading/subject.h"

// the working directory of tests are the bazel WORKSPACE root (in this case, src)
// this path is relative to the current working directory, i.e. the bazel root
constexpr const char* REPLAY_TEST_PATH_SUFFIX =
    "software/backend/replay_logging/test_replay";

namespace fs = std::experimental::filesystem;

// hahaha test subject because it's a subject but also a test subject
class TestSubject : public Subject<SensorProto>
{
   public:
    void sendValue(SensorProto val)
    {
        sendValueToObservers(val);
    }
};

TEST(ReplayTest, test_read_and_write_replay)
{
    // unfortunately due to the unavailablity of ordering tests and functions that
    // generate test files that actually work, we have to read recorded replays back,
    // write them, and then read them again in the same test
    // TODO: record replay_logging data with the game controller running , see #1584

    std::vector<SensorProto> read_replay_msgs;

    ReplayReader reader(fs::current_path() / REPLAY_TEST_PATH_SUFFIX);
    while (auto frame = reader.getNextMsg())
    {
        read_replay_msgs.emplace_back(*frame);
    }
    // do a quick sanity check here to assert that the timestamps are monotonically
    // increasing
    double max_timestamp = 0.f;
    for (const auto& msg : read_replay_msgs)
    {
        auto ts = msg.ssl_vision_msg().detection().t_sent();
        if (ts < max_timestamp)
        {
            FAIL() << "t_sent " << ts << " should have appeared after " << max_timestamp;
        }
        else
        {
            max_timestamp = ts;
        }
    }

    // write the read frames to another replay_logging directory
    auto output_path = fs::current_path() / "replaytest";
    std::shared_ptr<Observer<SensorProto>> logger_ptr =
        std::make_shared<ReplayLogger>(output_path, 1000);
    TestSubject subject;
    subject.registerObserver(logger_ptr);
    for (const auto msg : read_replay_msgs)
    {
        subject.sendValue(msg);
        // we have 3000 frames of replay_logging so we have to try to not fill the buffer
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // unfortunately we have to do this because there is otherwise no way to 'guarantee'
    // that the entire buffer of messages in the ThreadedObserver has been cleared and
    // written to disk by the ReplayLogger
    // This duration needs to be long enough to de facto make this a 'guarantee'
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // test that 3 files named "0", "1", and "2" are created in the output directory
    std::unordered_set<std::string> created_filenames;
    const std::unordered_set<std::string> expected_filenames = {"0", "1", "2"};

    for (const auto& dir_entry : fs::directory_iterator(output_path))
    {
        created_filenames.emplace(dir_entry.path().filename());
    }

    EXPECT_EQ(created_filenames, expected_filenames);

    // compare against the messages that we read
    std::vector<SensorProto> written_read_replay_msgs;
    ReplayReader reader2(output_path);
    while (auto frame = reader2.getNextMsg())
    {
        written_read_replay_msgs.emplace_back(*frame);
    }

    EXPECT_EQ(written_read_replay_msgs.size(), read_replay_msgs.size())
        << "Replay written and read back does not have the same number of frames!";

    for (size_t i = 0; i < written_read_replay_msgs.size(); i++)
    {
        bool eq = google::protobuf::util::MessageDifferencer::Equivalent(
            read_replay_msgs[i], written_read_replay_msgs[i]);
        EXPECT_TRUE(eq);
    }
}
