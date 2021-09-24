extern "C"
{
#include "firmware/app/logger/logger.h"

#include "proto/robot_log_msg.nanopb.h"
}

#include <gtest/gtest.h>

class FirmwareLoggerTest : public testing::Test
{
   public:
    static TbotsProto_RobotLog most_recent_robot_log;

    static void robotLogHandler(TbotsProto_RobotLog robot_log)
    {
        most_recent_robot_log = robot_log;
    }

    static TbotsProto_RobotLog getRecentlyHandledLog()
    {
        return most_recent_robot_log;
    }
};

TbotsProto_RobotLog FirmwareLoggerTest::most_recent_robot_log;

TEST_F(FirmwareLoggerTest, robot_0_log_info_macro_test)
{
    app_logger_init(0, &FirmwareLoggerTest::robotLogHandler);

    std::string expected_file_name = __FILE__;
    int expected_line_number       = __LINE__ + 2;

    TLOG_INFO("this is a test %d %x %s", 100, 15, "log");

    EXPECT_EQ(0, getRecentlyHandledLog().robot_id);
    EXPECT_EQ(TbotsProto_LogLevel_INFO, getRecentlyHandledLog().log_level);
    EXPECT_EQ("this is a test 100 f log", std::string(getRecentlyHandledLog().log_msg));
    EXPECT_EQ(expected_file_name, std::string(getRecentlyHandledLog().file_name));
    EXPECT_EQ(expected_line_number, getRecentlyHandledLog().line_number);
}

TEST_F(FirmwareLoggerTest, robot_4_log_warn_macro_test)
{
    app_logger_init(4, &FirmwareLoggerTest::robotLogHandler);

    std::string expected_file_name = __FILE__;
    int expected_line_number       = __LINE__ + 2;

    TLOG_WARNING("this is a test %d %x %s", 100, 15, "log");

    EXPECT_EQ(4, getRecentlyHandledLog().robot_id);
    EXPECT_EQ(TbotsProto_LogLevel_WARNING, getRecentlyHandledLog().log_level);
    EXPECT_EQ("this is a test 100 f log", std::string(getRecentlyHandledLog().log_msg));
    EXPECT_EQ(expected_file_name, std::string(getRecentlyHandledLog().file_name));
    EXPECT_EQ(expected_line_number, getRecentlyHandledLog().line_number);
}
