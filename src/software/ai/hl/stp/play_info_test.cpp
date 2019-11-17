#include "software/ai/hl/stp/play_info.h"

#include <gtest/gtest.h>

#include <string>

TEST(PlayInfoTest, set_and_get_play_type_works)
{
    PlayInfo test_play_info    = PlayInfo();
    std::string test_play_type = test_play_info.getPlayType();
    EXPECT_EQ(test_play_type, "");
    std::string s;
    s = "new type";
    test_play_info.setPlayType(s);
    test_play_type = test_play_info.getPlayType();
    EXPECT_EQ(test_play_type, "new type");
}

TEST(PlayInfoTest, set_and_get_play_name_works)
{
    PlayInfo test_play_info    = PlayInfo();
    std::string test_play_name = test_play_info.getPlayName();
    EXPECT_EQ(test_play_name, "");
    std::string s;
    s = "new name";
    test_play_info.setPlayName(s);
    test_play_name = test_play_info.getPlayName();
    EXPECT_EQ(test_play_name, "new name");
}

TEST(PlayInfoTest, set_and_get_play_robot_tactic_assignment_works)
{
    PlayInfo test_play_info                = PlayInfo();
    std::vector<std::string> test_play_rta = test_play_info.getRobotTacticAssignment();
    std::vector<std::string> emptyVec;
    emptyVec = {};
    EXPECT_EQ(test_play_rta, emptyVec);
    std::string s1, s2;
    s1 = "string one";
    s2 = "string two";
    std::vector<std::string> v;
    v = {s1, s2};
    test_play_info.setRobotTacticAssignment(v);
    test_play_rta = test_play_info.getRobotTacticAssignment();
    EXPECT_EQ(test_play_rta, v);
}

TEST(PlayInfoTest, add_assignment_test)
{
    PlayInfo test_play_info = PlayInfo();
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
    test_play_info.addAssignment(s1);
    EXPECT_EQ(test_play_info.getRobotTacticAssignment(), v1);
    test_play_info.addAssignment(s2);
    EXPECT_EQ(test_play_info.getRobotTacticAssignment(), v2);
    test_play_info.addAssignment(s3);
    EXPECT_EQ(test_play_info.getRobotTacticAssignment(), v3);
}
