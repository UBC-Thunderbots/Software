#include "ai/world/field.h"

#include <gtest/gtest.h>

class FieldTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        length               = 9.0;
        width                = 6.0;
        defense_length       = 1.0;
        defense_width        = 2.0;
        goal_width           = 1.0;
        boundary_width       = 0.3;
        center_circle_radius = 0.5;

        field = Field(length, width, defense_length, defense_width, goal_width,
                      boundary_width, center_circle_radius);
    }

    Field field = Field(0, 0, 0, 0, 0, 0, 0);
    double length;
    double width;
    double defense_length;
    double defense_width;
    double goal_width;
    double boundary_width;
    double center_circle_radius;
};

TEST_F(FieldTest, construct_with_parameters)
{
    // The field was already constructed in the test setup, so we only need to check
    // values here
    EXPECT_DOUBLE_EQ(length, field.length());
    EXPECT_DOUBLE_EQ(width, field.width());
    EXPECT_DOUBLE_EQ(goal_width, field.goalWidth());
    EXPECT_DOUBLE_EQ(center_circle_radius, field.centreCircleRadius());
    EXPECT_DOUBLE_EQ(defense_width, field.defenseAreaWidth());
    EXPECT_DOUBLE_EQ(defense_length, field.defenseAreaLength());
}

TEST_F(FieldTest, update_with_all_parameters)
{
    Field field_to_update = Field(0, 0, 0, 0, 0, 0, 0);

    field_to_update.updateDimensions(length, width, defense_length, defense_width,
                                     goal_width, boundary_width, center_circle_radius);

    EXPECT_DOUBLE_EQ(9.6, field_to_update.totalLength());
    EXPECT_DOUBLE_EQ(6.6, field_to_update.totalWidth());
    EXPECT_DOUBLE_EQ(0.3, field_to_update.boundaryWidth());

    EXPECT_EQ(Point(-4.5, 0.0), field_to_update.friendlyGoal());
    EXPECT_EQ(Point(4.5, 0.0), field_to_update.enemyGoal());

    EXPECT_EQ(Point(-4.5, 0.5), field_to_update.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field_to_update.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field_to_update.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field_to_update.enemyGoalpostNeg());

    EXPECT_EQ(Rectangle(Point(-4.5, 1.0), Point(-3.5, -1.0)),
              field_to_update.friendlyDefenseArea());
    EXPECT_EQ(Rectangle(Point(4.5, 1.0), Point(3.5, -1.0)),
              field_to_update.enemyDefenseArea());

    EXPECT_EQ(Point(-3.5, 0.0), field_to_update.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field_to_update.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field_to_update.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field_to_update.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field_to_update.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field_to_update.enemyCornerNeg());
}

TEST_F(FieldTest, update_with_new_field)
{
    Field field_to_update = Field(0, 0, 0, 0, 0, 0, 0);

    field_to_update.updateDimensions(field);

    EXPECT_DOUBLE_EQ(9.6, field_to_update.totalLength());
    EXPECT_DOUBLE_EQ(6.6, field_to_update.totalWidth());
    EXPECT_DOUBLE_EQ(0.3, field_to_update.boundaryWidth());

    EXPECT_EQ(Point(-4.5, 0.0), field_to_update.friendlyGoal());
    EXPECT_EQ(Point(4.5, 0.0), field_to_update.enemyGoal());

    EXPECT_EQ(Point(-4.5, 0.5), field_to_update.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field_to_update.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field_to_update.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field_to_update.enemyGoalpostNeg());

    EXPECT_EQ(Rectangle(Point(-4.5, 1.0), Point(-3.5, -1.0)),
              field_to_update.friendlyDefenseArea());
    EXPECT_EQ(Rectangle(Point(4.5, 1.0), Point(3.5, -1.0)),
              field_to_update.enemyDefenseArea());

    EXPECT_EQ(Point(-3.5, 0.0), field_to_update.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field_to_update.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field_to_update.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field_to_update.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field_to_update.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field_to_update.enemyCornerNeg());
}

TEST_F(FieldTest, equality_operator_fields_with_different_lengths)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    Field field_2 = Field(length / 2, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}
TEST_F(FieldTest, equality_operator_fields_with_different_widths)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    Field field_2 = Field(length, width * 2, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_defense_length)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    Field field_2 = Field(length, width, defense_length * 2, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_defense_width)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    Field field_2 = Field(length, width, defense_length, defense_width / 2, goal_width,
                          boundary_width, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_goal_width)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    Field field_2 = Field(length, width, defense_length, defense_width, 0, boundary_width,
                          center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_boundary_width)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    Field field_2 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width * 1.1, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_center_circle_radius)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius);

    Field field_2 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius * 10);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, ball_not_in_defense_area)
{
    // point around centre
    Point p(2, 3);
    EXPECT_EQ(false, field.pointInFriendlyDefenseArea(p));
    EXPECT_EQ(false, field.pointInEnemyDefenseArea(p));
}

TEST_F(FieldTest, ball_in_friendly_defense_area)
{
    Point p(-4, 0.5);
    EXPECT_EQ(true, field.pointInFriendlyDefenseArea(p));
    EXPECT_EQ(false, field.pointInEnemyDefenseArea(p));
}

TEST_F(FieldTest, ball_in_enemy_defense_area)
{
    Point p(4, -1.0);
    EXPECT_EQ(false, field.pointInFriendlyDefenseArea(p));
    EXPECT_EQ(true, field.pointInEnemyDefenseArea(p));
}

TEST_F(FieldTest, ball_just_outside_enemy_defense_area)
{
    Point p(4, -1.5);
    EXPECT_EQ(false, field.pointInFriendlyDefenseArea(p));
    EXPECT_EQ(false, field.pointInEnemyDefenseArea(p));
}

TEST_F(FieldTest, ball_just_outside_friendly_defense_area)
{
    Point p(-2, -.5);
    EXPECT_EQ(false, field.pointInFriendlyDefenseArea(p));
    EXPECT_EQ(false, field.pointInEnemyDefenseArea(p));
}
