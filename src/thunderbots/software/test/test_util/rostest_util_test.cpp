#include "test/test_util/rostest_util.h"

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

/*
 * Unit tests for the rostest utilities
 */

class RosTestUtilsTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        // Enable latching for this publisher so that if messages are published before
        // the subscriber from the waitForMessageOnTopic function is connected, it will
        // still receive the message. Otherwise the function may "miss" the message and
        // the tests will fail
        test_publisher = node_handle.advertise<std_msgs::String>(topic_name, 1, true);

        one_second  = Duration::fromSeconds(1);
        ten_seconds = Duration::fromSeconds(10);
    }

    ros::NodeHandle node_handle;
    ros::Publisher test_publisher;

    Duration one_second;
    Duration ten_seconds;

    const std::string topic_name = "/rostest_utils_test";
};

TEST_F(RosTestUtilsTest, test_publish_and_receive_message_within_timeout)
{
    std_msgs::String msg;
    msg.data = "C++ is my favorite language";

    test_publisher.publish(msg);
    auto received_message =
        RosTestUtil::waitForMessageOnTopic<std_msgs::String::ConstPtr>(
            node_handle, topic_name, ten_seconds);

    EXPECT_EQ(msg.data, received_message->data);
}

TEST_F(RosTestUtilsTest, test_do_not_receive_message_within_timeout)
{
    // We don't publish a message here so that no message will be received by the waiting
    // function
    EXPECT_THROW(RosTestUtil::waitForMessageOnTopic<std_msgs::String::ConstPtr>(
                     node_handle, topic_name, one_second),
                 std::runtime_error);
}

int main(int argc, char **argv)
{
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "rostest_util_test");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
