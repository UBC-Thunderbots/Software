#include "ai/world/field.h"
#include <gtest/gtest.h>

TEST(FieldTest, construction)
{
    Field field = Field();

    EXPECT_FALSE(field.valid());
    EXPECT_DOUBLE_EQ(0, field.length());
    EXPECT_DOUBLE_EQ(0, field.width());
    EXPECT_DOUBLE_EQ(0, field.goalWidth());
    EXPECT_DOUBLE_EQ(0, field.centreCircleRadius());
    EXPECT_DOUBLE_EQ(0, field.defenseAreaWidth());
    EXPECT_DOUBLE_EQ(0, field.defenseAreaLength());
}

TEST(FieldTest, update_with_all_parameters)
{
    Field field                 = Field();
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    EXPECT_TRUE(field.valid());

    EXPECT_DOUBLE_EQ(9.6, field.totalLength());
    EXPECT_DOUBLE_EQ(6.6, field.totalWidth());
    EXPECT_DOUBLE_EQ(0.3, field.boundaryWidth());

    EXPECT_EQ(Point(-4.5, 0.0), field.friendlyGoal());
    EXPECT_EQ(Point(4.5, 0.0), field.enemyGoal());

    EXPECT_EQ(Point(-4.5, 0.5), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field.enemyGoalpostNeg());

    EXPECT_EQ(Rect(Point(-4.5, 1.0), Point(-3.5, -1.0)), field.friendlyDefenseArea());
    EXPECT_EQ(Rect(Point(4.5, 1.0), Point(3.5, -1.0)), field.enemyDefenseArea());

    EXPECT_EQ(Point(-3.5, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field.enemyCornerNeg());
}

TEST(FieldTest, update_specific_params)
{
    Field field                 = Field();
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    field.updateDimensions(field.length(), width / 2, field.defenseAreaLength(),
                           defense_width * 1.5, field.goalWidth(), boundary_width * 0,
                           field.centreCircleRadius());

    EXPECT_TRUE(field.valid());

    EXPECT_DOUBLE_EQ(9.0, field.totalLength());
    EXPECT_DOUBLE_EQ(3.0, field.totalWidth());
    EXPECT_DOUBLE_EQ(0.0, field.boundaryWidth());

    EXPECT_EQ(Point(-4.5, 0.0), field.friendlyGoal());
    EXPECT_EQ(Point(4.5, 0.0), field.enemyGoal());

    EXPECT_EQ(Point(-4.5, 0.5), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field.enemyGoalpostNeg());

    EXPECT_EQ(Rect(Point(-4.5, 1.5), Point(-3.5, -1.5)), field.friendlyDefenseArea());
    EXPECT_EQ(Rect(Point(4.5, 1.5), Point(3.5, -1.5)), field.enemyDefenseArea());

    EXPECT_EQ(Point(-3.5, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 1.5), field.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -1.5), field.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 1.5), field.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -1.5), field.enemyCornerNeg());
}

TEST(FieldTest, update_specific_params_2)
{
    Field field                 = Field();
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    field.updateDimensions(field.length() / 3, field.width(),
                           field.defenseAreaLength() * 0.5, field.defenseAreaWidth(),
                           field.goalWidth() * 2, field.boundaryWidth(),
                           field.centreCircleRadius() * 1.25);

    EXPECT_TRUE(field.valid());

    EXPECT_DOUBLE_EQ(3.6, field.totalLength());
    EXPECT_DOUBLE_EQ(6.6, field.totalWidth());
    EXPECT_DOUBLE_EQ(0.3, field.boundaryWidth());

    EXPECT_EQ(Point(-1.5, 0.0), field.friendlyGoal());
    EXPECT_EQ(Point(1.5, 0.0), field.enemyGoal());

    EXPECT_EQ(Point(-1.5, 1.0), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-1.5, -1.0), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(1.5, 1.0), field.enemyGoalpostPos());
    EXPECT_EQ(Point(1.5, -1.0), field.enemyGoalpostNeg());

    EXPECT_EQ(Rect(Point(-1.5, 1.0), Point(-1.0, -1.0)), field.friendlyDefenseArea());
    EXPECT_EQ(Rect(Point(1.5, 1.0), Point(1.0, -1.0)), field.enemyDefenseArea());

    EXPECT_EQ(Point(-1.0, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(1.0, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-1.5, 3.0), field.friendlyCornerPos());
    EXPECT_EQ(Point(-1.5, -3.0), field.friendlyCornerNeg());
    EXPECT_EQ(Point(1.5, 3.0), field.enemyCornerPos());
    EXPECT_EQ(Point(1.5, -3.0), field.enemyCornerNeg());
}

TEST(FieldTest, update_with_new_field)
{
    Field field_update          = Field();
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    field_update.updateDimensions(length, width, defense_length, defense_width,
                                  goal_width, boundary_width, center_circle_radius);

    Field field = Field();

    field.updateDimensions(field_update);

    EXPECT_TRUE(field.valid());

    EXPECT_DOUBLE_EQ(9.6, field.totalLength());
    EXPECT_DOUBLE_EQ(6.6, field.totalWidth());
    EXPECT_DOUBLE_EQ(0.3, field.boundaryWidth());

    EXPECT_EQ(Point(-4.5, 0.0), field.friendlyGoal());
    EXPECT_EQ(Point(4.5, 0.0), field.enemyGoal());

    EXPECT_EQ(Point(-4.5, 0.5), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field.enemyGoalpostNeg());

    EXPECT_EQ(Rect(Point(-4.5, 1.0), Point(-3.5, -1.0)), field.friendlyDefenseArea());
    EXPECT_EQ(Rect(Point(4.5, 1.0), Point(3.5, -1.0)), field.enemyDefenseArea());

    EXPECT_EQ(Point(-3.5, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field.enemyCornerNeg());
}

TEST(FieldTest, equality_operators)
{
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    Field invalid_field = Field();

    Field field = Field();
    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    Field field_other = Field();
    field_other.updateDimensions(length, width, defense_length, defense_width, goal_width,
                                 boundary_width, center_circle_radius);

    // field and field_other have been updated with the same dimensions, so
    // should be equal
    EXPECT_EQ(invalid_field, invalid_field);
    EXPECT_NE(invalid_field, field);
    EXPECT_NE(invalid_field, field_other);
    EXPECT_EQ(field, field_other);
    EXPECT_EQ(field, field);

    // Update with the same dimensions as above, except for a slightly larger length
    field_other.updateDimensions(length + 0.02, width, defense_length, goal_width,
                                 defense_width, boundary_width, center_circle_radius);

    EXPECT_NE(field, field_other);
    EXPECT_NE(field_other, field);
    EXPECT_NE(invalid_field, field_other);
    EXPECT_NE(field_other, invalid_field);
    EXPECT_EQ(field_other, field_other);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
