#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include "software/multithreading/subject.h"
#include "software/replay/replay_logger.h"
#include "software/replay/replay_reader.h"

constexpr const char* REPLAY_TEST_PATH = "/tmp/replaytest";

namespace fs = std::experimental::filesystem;

class ReplayTest : public ::testing::Test
{
    void SetUp() override
    {
        fs::path replay_test_path(REPLAY_TEST_PATH);
        std::cout << std::boolalpha;

        if (fs::exists(replay_test_path))
        {
            std::cout << "deleted " << fs::remove_all(REPLAY_TEST_PATH)
                      << " files from test replay dir" << std::endl;
        }
    }
};

// hahaha test subject because it's a subject but also a test subject
class TestSubject : public Subject<TbotsSensorProto>
{
   public:
    void sendValue(TbotsSensorProto val)
    {
        sendValueToObservers(val);
    }
};

TbotsSensorProto generateTestFrame(int value)
{
    TbotsSensorProto test_frame;
    SSL_DetectionBall test_ball;
    test_ball.set_area(value);
    test_ball.set_confidence(0.5);
    test_ball.set_pixel_x(420);
    test_ball.set_pixel_y(69420);
    test_ball.set_x(123);
    test_ball.set_y(456);
    test_ball.set_z(789);
    test_frame.ssl_wrapperpacket().InitAsDefaultInstance();
    test_frame.mutable_ssl_wrapperpacket()->mutable_detection()->mutable_balls()->Add(
        dynamic_cast<SSL_DetectionBall&&>(test_ball));
    test_frame.mutable_ssl_wrapperpacket()->mutable_detection()->set_frame_number(value);
    test_frame.mutable_ssl_wrapperpacket()->mutable_detection()->set_t_capture(value);
    test_frame.mutable_ssl_wrapperpacket()->mutable_detection()->set_t_sent(value);
    test_frame.mutable_ssl_wrapperpacket()->mutable_detection()->set_camera_id(value);
    return test_frame;
}

TEST_F(ReplayTest, test_single_segment_single_message)
{
    TbotsSensorProto test_frame = generateTestFrame(5);

    {
        TestSubject subject;
        auto replay_logger_ptr =
            std::make_shared<ReplayLogger>(std::string(REPLAY_TEST_PATH), 1);
        subject.registerObserver(replay_logger_ptr);

        subject.sendValue(test_frame);
    }

    {
        ReplayReader reader(REPLAY_TEST_PATH);
        auto frame = reader.getNextFrame();
        if (!frame)
            FAIL();
        EXPECT_TRUE(google::protobuf::util::MessageDifferencer::ApproximatelyEquivalent(
            test_frame.ssl_wrapperpacket(), *frame));
    }
}

TEST_F(ReplayTest, test_single_segment_multiple_message)
{
    constexpr int FRAME_COUNT = 5;

    std::vector<TbotsSensorProto> test_messages;
    for (int i = 0; i < FRAME_COUNT; i++)
    {
        test_messages.push_back(generateTestFrame(i));
    }

    {
        TestSubject subject;
        auto replay_logger_ptr =
            std::make_shared<ReplayLogger>(std::string(REPLAY_TEST_PATH), 200);
        subject.registerObserver(replay_logger_ptr);

        for (auto message : test_messages)
        {
            subject.sendValue(message);
        }
    }

    {
        ReplayReader reader(REPLAY_TEST_PATH);
        for (auto expected_message : test_messages)
        {
            auto actual_message_or_null = reader.getNextFrame();
            if (!actual_message_or_null)
                FAIL();
            EXPECT_TRUE(
                google::protobuf::util::MessageDifferencer::ApproximatelyEquivalent(
                    expected_message, *actual_message_or_null));
        }
    }
}

TEST_F(ReplayTest, test_multiple_segment_multiple_message)
{
    constexpr int FRAME_COUNT = 200;

    std::vector<TbotsSensorProto> test_messages;
    for (int i = 0; i < FRAME_COUNT; i++)
    {
        test_messages.push_back(generateTestFrame(i));
    }

    {
        TestSubject subject;
        auto replay_logger_ptr =
            std::make_shared<ReplayLogger>(std::string(REPLAY_TEST_PATH), 5);
        subject.registerObserver(replay_logger_ptr);

        for (auto message : test_messages)
        {
            subject.sendValue(message);
        }
    }

    {
        ReplayReader reader(REPLAY_TEST_PATH);
        for (auto expected_message : test_messages)
        {
            auto actual_message_or_null = reader.getNextFrame();
            if (!actual_message_or_null)
                FAIL();

            std::cout << "ACTUAL: " << actual_message_or_null->ssl_wrapperpacket().detection().t_sent()
                      << " EXPECTED: " << expected_message.ssl_wrapperpacket().detection().t_sent()
                      << std::endl;
        }
    }
}
