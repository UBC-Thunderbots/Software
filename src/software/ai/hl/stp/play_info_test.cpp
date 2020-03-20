#include "software/ai/hl/stp/play_info.h"

#include <gtest/gtest.h>

#include <string>

TEST(PlayInfoTest, get_play_type_works)
{
    std::string s, empty;
    s                            = "new type";
    empty                        = "";
    PlayInfo test_play_info      = PlayInfo();
    PlayInfo custom_play_info    = PlayInfo(s, empty, {});
    std::string test_play_type   = test_play_info.getPlayType();
    std::string custom_play_type = custom_play_info.getPlayType();
    EXPECT_EQ(test_play_type, "");
    EXPECT_EQ(custom_play_type, "new type");
}

TEST(PlayInfoTest, get_play_name_works)
{
    std::string s                = "new name";
    std::string empty            = "";
    PlayInfo test_play_info      = PlayInfo();
    PlayInfo custom_play_info    = PlayInfo(empty, s, {});
    std::string test_play_name   = test_play_info.getPlayName();
    std::string custom_play_name = custom_play_info.getPlayName();
    EXPECT_EQ(test_play_name, "");
    EXPECT_EQ(custom_play_name, "new name");
}

TEST(PlayInfoTest, get_play_robot_tactic_assignment_works)
{
    std::string s1, s2, emptyString;
    emptyString = "";
    s1          = "string one";
    s2          = "string two";
    std::unordered_set<std::string> emptySet, v;
    emptySet                  = {};
    v                         = {s1, s2};
    PlayInfo test_play_info   = PlayInfo();
    PlayInfo custom_play_info = PlayInfo(emptyString, emptyString, v);
    std::unordered_set<std::string> test_play_rta =
        test_play_info.getRobotTacticAssignment();
    std::unordered_set<std::string> custom_play_rta =
        custom_play_info.getRobotTacticAssignment();
    EXPECT_EQ(test_play_rta, emptySet);
    EXPECT_EQ(custom_play_rta, v);
}

TEST(PlayInfoTest, add_assignment_test)
{
    PlayInfo test_play_info = PlayInfo();
    std::string s1, s2, s3;
    s1 = "string one";
    s2 = "string two";
    s3 = "string three";
    std::unordered_set<std::string> v, v1, v2, v3;
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
    PlayInfo test_play_info1, test_play_info2, test_play_info3;
    std::unordered_set<std::string> emptySet;
    std::string s1, s2;
    s1              = "string one";
    s2              = "string two";
    emptySet        = {};
    test_play_info1 = PlayInfo(s1, s2, emptySet);
    test_play_info2 = PlayInfo(s1, s2, emptySet);
    test_play_info3 = PlayInfo(s1, s1, emptySet);
    EXPECT_TRUE(test_play_info1 == test_play_info2);
    EXPECT_FALSE(test_play_info1 == test_play_info3);
}
