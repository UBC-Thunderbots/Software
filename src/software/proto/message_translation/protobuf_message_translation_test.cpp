#include <gtest/gtest.h>
#include <string.h>
#include "shared/proto/geometry.pb.h"
#include "shared/proto/tbots_software_msgs.pb.h"
#include "shared/proto/vision.pb.h"
#include "software/primitive/primitive.h"
#include "software/world/world.h"
#include "software/proto/message_translation/protobuf_message_translation.h"
#include "software/test_util/test_util.h"

class ProtobufTranslationTest : public ::testing::Test
{
    public:
        static void assertPointMessageEqual(const Point &point, std::unique_ptr<PointMsg> point_msg)
        {
            ASSERT_TRUE(point_msg->x() == point.x());
            ASSERT_TRUE(point_msg->y() == point.y());
        }

        static void assertAngleMessageEqual(const Angle &angle, std::unique_ptr<AngleMsg> angle_msg)
        {
            ASSERT_TRUE(angle_msg->radians() == angle.toRadians());
        }

        static void assertVectorMessageEqual(const Vector &vector, std::unique_ptr<VectorMsg> vector_msg)
        {
            ASSERT_TRUE(vector_msg->x_component() == vector.x());
            ASSERT_TRUE(vector_msg->y_component() == vector.y());
        }

        static void assertSaneTimestamp(std::unique_ptr<TimestampMsg> timestamp){

            // time will only move forward, we make sure the number that was assigned is strictly
            // from the past
            ASSERT_TRUE(timestamp->epoch_timestamp_seconds() <= std::time(0));
        }
};


TEST(ProtobufTranslationTest, point_msg_test)
{
    auto point = Point(420, 420);
    auto point_msg = ProtobufMessageTranslation::getPointMsgFromPoint(point);

    ProtobufTranslationTest::assertPointMessageEqual(point, std::move(point_msg));
}

TEST(ProtobufTranslationTest, angle_msg_test)
{
    auto angle = Angle::fromRadians(420);
    auto angle_msg = ProtobufMessageTranslation::getAngleMsgFromAngle(angle);

    ProtobufTranslationTest::assertAngleMessageEqual(angle, std::move(angle_msg));
}

TEST(ProtobufTranslationTest, vector_msg_test)
{
    auto vector = Vector(420, 420);
    auto vector_msg = ProtobufMessageTranslation::getVectorMsgFromVector(vector);

    ProtobufTranslationTest::assertVectorMessageEqual(vector, std::move(vector_msg));
}

TEST(ProtobufTranslationTest, timestamp_msg_test)
{
    auto timestamp_msg = ProtobufMessageTranslation::getCurrentTimestampMsg();
    ProtobufTranslationTest::assertSaneTimestamp(std::move(timestamp_msg));
}

TEST(ProtobufTranslationTest, robot_state_msg_test){

    //Robot robot(0, Point(420, 420), Velocity(420, 420), Angle::fromRadians(420), Angle::fromRadians(420), Timestamp::fromSeconds(0));

    //auto robot_state_msg = getRobotStateMsgFromRobot(const Robot& robot);

}

