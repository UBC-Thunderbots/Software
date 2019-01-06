#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

#include <string>

#include "test/test_util/rostest_util.h"
#include "util/logger/init.h"

class LoggerTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        // Initialize the logger
        Util::Logger::LoggerSingleton::initializeLogger();
    }

    ros::NodeHandle nh_;
};

TEST_F(LoggerTest, test_log_messages_with_INFO_severity_are_sent_to_rosout_topic)
{
    std::string msg_data = "INFO log test message";

    LOG(INFO) << msg_data << std::endl;

    // We MUST use the /rosout topic rather than the commonly suggested /rosout_agg,
    // because messages are not re-published to /rosout_agg during rostests
    auto message =
        RosTestUtil::waitForMessageOnTopic<rosgraph_msgs::Log::ConstPtr>(nh_, "/rosout");

    std::string log_output                    = message->msg;
    rosgraph_msgs::Log::_level_type log_level = message->level;

    // Because messages published on /rosout also contain other metadata (such as a
    // timestamp, the name of the file where the message originated etc.), we just check
    // that our initial log message is contained in the /rosout output
    bool log_output_contains_msg_data = log_output.find(msg_data) != std::string::npos;

    EXPECT_TRUE(log_output_contains_msg_data);
    EXPECT_EQ(rosgraph_msgs::Log::INFO, log_level);
}

TEST_F(LoggerTest, test_log_messages_with_DEBUG_severity_are_sent_to_rosout_topic)
{
    std::string msg_data = "DEBUG log test message";

    LOG(DEBUG) << msg_data << std::endl;

    // We MUST use the /rosout topic rather than the commonly suggested /rosout_agg,
    // because messages are not re-published to /rosout_agg during rostests
    auto message =
        RosTestUtil::waitForMessageOnTopic<rosgraph_msgs::Log::ConstPtr>(nh_, "/rosout");

    std::string log_output                    = message->msg;
    rosgraph_msgs::Log::_level_type log_level = message->level;

    // Because messages published on /rosout also contain other metadata (such as a
    // timestamp, the name of the file where the message originated etc.), we just check
    // that our initial log message is contained in the /rosout output
    bool log_output_contains_msg_data = log_output.find(msg_data) != std::string::npos;

    EXPECT_TRUE(log_output_contains_msg_data);
    EXPECT_EQ(rosgraph_msgs::Log::DEBUG, log_level);
}

TEST_F(LoggerTest, test_log_messages_with_WARNING_severity_are_sent_to_rosout_topic)
{
    std::string msg_data = "WARNING log test message";

    LOG(WARNING) << msg_data << std::endl;

    // We MUST use the /rosout topic rather than the commonly suggested /rosout_agg,
    // because messages are not re-published to /rosout_agg during rostests
    auto message =
        RosTestUtil::waitForMessageOnTopic<rosgraph_msgs::Log::ConstPtr>(nh_, "/rosout");

    std::string log_output                    = message->msg;
    rosgraph_msgs::Log::_level_type log_level = message->level;

    // Because messages published on /rosout also contain other metadata (such as a
    // timestamp, the name of the file where the message originated etc.), we just check
    // that our initial log message is contained in the /rosout output
    bool log_output_contains_msg_data = log_output.find(msg_data) != std::string::npos;

    EXPECT_TRUE(log_output_contains_msg_data);
    EXPECT_EQ(rosgraph_msgs::Log::WARN, log_level);
}

int main(int argc, char **argv)
{
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "logger_test");
    ::testing::InitGoogleTest(&argc, argv);
    // Because of https://github.com/ros/ros_comm/issues/688 we create an extra NodeHandle
    // here that will stay in scope until all the tests have been completed. Otherwise
    // once the NodeHandle in the first test that is run goes out of scope, the rosconsole
    // functions (ROS_INFO, etc.) will no longer work, and therefore the logger will no
    // longer work and the tests will fail
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
