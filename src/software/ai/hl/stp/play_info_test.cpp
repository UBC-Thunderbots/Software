#include "software/ai/hl/stp/play_info.h"

#include <gtest/gtest.h>

#include <string>

TEST(PlayInfoTest, get_referee_command_name_works)
{
    std::string s, empty;
    s                                       = "new type";
    empty                                   = "";
    PlayInfo custom_play_info               = PlayInfo(s, empty, {});
    std::string custom_referee_command_name = custom_play_info.getRefereeCommandName();
    EXPECT_EQ(custom_referee_command_name, "new type");
}

TEST(PlayInfoTest, get_play_name_works)
{
    std::string s                = "new name";
    std::string empty            = "";
    PlayInfo custom_play_info    = PlayInfo(empty, s, {});
    std::string custom_play_name = custom_play_info.getPlayName();
    EXPECT_EQ(custom_play_name, "new name");
}

TEST(PlayInfoTest, get_play_robot_tactic_assignment_works)
{
    std::string s1, s2, emptyString;
    emptyString = "";
    s1          = "string one";
    s2          = "string two";
    std::vector<std::string> v;
    v                         = {s1, s2};
    PlayInfo custom_play_info = PlayInfo(emptyString, emptyString, v);
    std::vector<std::string> custom_play_rta =
        custom_play_info.getRobotTacticAssignment();
    EXPECT_EQ(custom_play_rta, v);
}

TEST(PlayInfoTest, add_assignment_test)
{
    PlayInfo test_play_info = PlayInfo("test_command", "test_play", {});
    std::string s1, s2, s3;
    s1 = "string one";
    s2 = "string two";
    s3 = "string three";
    std::vector<std::string> v, v1, v2, v3;
    v  = {};
    v1 = {s1};
    v2 = {s1, s2};
    v3 = {s1, s2, s3};
    EXPECT_EQ(test_play_info.getRobotTacticAssignment(), v);
    test_play_info.addRobotTacticAssignment(s1);
    EXPECT_EQ(test_play_info.getRobotTacticAssignment(), v1);
    test_play_info.addRobotTacticAssignment(s2);
    EXPECT_EQ(test_play_info.getRobotTacticAssignment(), v2);
    test_play_info.addRobotTacticAssignment(s3);
    EXPECT_EQ(test_play_info.getRobotTacticAssignment(), v3);
}

TEST(PlayInfoTest, equality_operator_works)
{
    std::vector<std::string> empty_vector = {};
    std::string s1, s2;
    s1 = "string one";
    s2 = "string two";
    PlayInfo test_play_info1(s1, s2, empty_vector);
    PlayInfo test_play_info2(s1, s2, empty_vector);
    PlayInfo test_play_info3(s1, s1, empty_vector);
    EXPECT_TRUE(test_play_info1 == test_play_info2);
    EXPECT_FALSE(test_play_info1 == test_play_info3);
}
