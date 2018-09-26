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

TEST(FieldTest, equality_operator_fields_with_different_lengths)
{
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    Field field = Field();
    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    Field field_other = Field();
    field_other.updateDimensions(length / 2, width, defense_length, defense_width,
                                 goal_width, boundary_width, center_circle_radius);

    EXPECT_NE(field, field_other);
}
TEST(FieldTest, equality_operator_fields_with_different_widths)
{
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    Field field = Field();
    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    Field field_other = Field();
    field_other.updateDimensions(length, width * 2, defense_length, defense_width,
                                 goal_width, boundary_width, center_circle_radius);

    EXPECT_NE(field, field_other);
}

TEST(FieldTest, equality_operator_fields_with_different_defense_length)
{
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    Field field = Field();
    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    Field field_other = Field();
    field_other.updateDimensions(length, width, defense_length * 2, defense_width,
                                 goal_width, boundary_width, center_circle_radius);

    EXPECT_NE(field, field_other);
}

TEST(FieldTest, equality_operator_fields_with_different_defense_width)
{
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    Field field = Field();
    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    Field field_other = Field();
    field_other.updateDimensions(length, width, defense_length, defense_width / 2,
                                 goal_width, boundary_width, center_circle_radius);

    EXPECT_NE(field, field_other);
}

TEST(FieldTest, equality_operator_fields_with_different_goal_width)
{
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    Field field = Field();
    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    Field field_other = Field();
    field_other.updateDimensions(length, width, defense_length, defense_width, 0,
                                 boundary_width, center_circle_radius);

    EXPECT_NE(field, field_other);
}

TEST(FieldTest, equality_operator_fields_with_different_boundary_width)
{
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    Field field = Field();
    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    Field field_other = Field();
    field_other.updateDimensions(length, width, defense_length, defense_width, goal_width,
                                 boundary_width * 1.1, center_circle_radius);

    EXPECT_NE(field, field_other);
}

TEST(FieldTest, equality_operator_fields_with_different_center_circle_radius)
{
    double length               = 9.0;
    double width                = 6.0;
    double defense_width        = 2.0;
    double defense_length       = 1.0;
    double goal_width           = 1.0;
    double boundary_width       = 0.3;
    double center_circle_radius = 0.5;

    Field field = Field();
    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius);

    Field field_other = Field();
    field_other.updateDimensions(length, width, defense_length, defense_width, goal_width,
                                 boundary_width, center_circle_radius * 10);

    EXPECT_NE(field, field_other);
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
