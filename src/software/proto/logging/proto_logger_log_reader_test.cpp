#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/multithreading/subject.h"
#include "software/proto/logging/proto_log_reader.h"
#include "software/proto/logging/proto_logger.h"
#include "software/proto/sensor_msg.pb.h"

// the working directory of tests are the bazel WORKSPACE root (in this case, src)
// this path is relative to the current working directory, i.e. the bazel root
constexpr const char* REPLAY_TEST_PATH_SUFFIX = "software/proto/logging/test_logs";

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

TEST(ProtoLoggerLogReaderTest, test_read_and_write_proto_log)
{
    // unfortunately due to the unavailablity of ordering tests and functions that
    // generate test files that actually work, we have to read recorded replays back,
    // write them, and then read them again in the same test

    static constexpr size_t MSGS_PER_CHUNK = 1000;

    std::vector<SensorProto> read_replay_msgs;

    ProtoLogReader reader(fs::current_path() / REPLAY_TEST_PATH_SUFFIX);
    while (auto frame = reader.getNextMsg<SensorProto>())
    {
        read_replay_msgs.emplace_back(*frame);
    }
    // do a quick sanity check here to assert that the timestamps are monotonically
    // increasing
    double max_timestamp = 0.f;
    // idx is not used for testing of any sort but it will be useful if the test below
    // fails and you need to debug it using e.g. python protobuf
    size_t idx = 0;
    for (const auto& msg : read_replay_msgs)
    {
        auto ts = msg.backend_received_time().epoch_timestamp_seconds();
        if (ts < max_timestamp)
        {
            FAIL() << "msg idx=" << idx << " with time_received=" << ts
                   << " should have appeared after " << max_timestamp;
        }
        else
        {
            max_timestamp = ts;
        }
        idx++;
    }

    // write the read frames to another replay_logging directory
    auto output_path = fs::current_path() / "replaytest";
    std::shared_ptr<Observer<SensorProto>> logger_ptr =
        std::make_shared<ProtoLogger<SensorProto>>(
            output_path, MSGS_PER_CHUNK,
            [](const SensorProto& lhs, const SensorProto& rhs) {
                return lhs.backend_received_time().epoch_timestamp_seconds() <
                       rhs.backend_received_time().epoch_timestamp_seconds();
            });
    TestSubject subject;
    subject.registerObserver(logger_ptr);

    // unfortunately we have to do this due to the performance of the CI node since
    // the sort that happens right before a chunk is saved to disk is time consuming
    // and there's still messages in the buffer left to process
    for (size_t i = 0; i < read_replay_msgs.size(); i++)
    {
        auto& msg = read_replay_msgs[i];
        subject.sendValue(msg);
        LOG(INFO) << "Sent message with timestamp "
                  << msg.backend_received_time().epoch_timestamp_seconds();

        if (i % MSGS_PER_CHUNK == MSGS_PER_CHUNK - 1)
        {
            // we just sent the last message in a chunk, sleep for longer
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

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
    ProtoLogReader reader2(output_path);
    while (auto frame = reader2.getNextMsg<SensorProto>())
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
