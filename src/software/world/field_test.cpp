#include "software/world/field.h"

#include <gtest/gtest.h>

#include "shared/constants.h"

class FieldTest : public ::testing::Test
{
   public:
    FieldTest()
        : x_length(9.0),
          y_length(6.0),
          defense_x_length(1.0),
          defense_y_length(2.0),
          goal_x_length(0.18),
          goal_y_length(1.0),
          boundary_buffer_size(0.3),
          center_circle_radius(0.5),
          field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
                goal_y_length, boundary_buffer_size, center_circle_radius)
    {
    }

   protected:
    double x_length;
    double y_length;
    double defense_x_length;
    double defense_y_length;
    double goal_x_length;
    double goal_y_length;
    double boundary_buffer_size;
    double center_circle_radius;
    Field field;
};

TEST_F(FieldTest, construct_with_parameters)
{
    // The field was already constructed in the test setup, so we only need to check
    // values here
    EXPECT_DOUBLE_EQ(x_length, field.xLength());
    EXPECT_DOUBLE_EQ(y_length, field.yLength());
    EXPECT_DOUBLE_EQ(goal_y_length, field.goalYLength());
    EXPECT_DOUBLE_EQ(goal_x_length, field.goalXLength());
    EXPECT_DOUBLE_EQ(center_circle_radius, field.centerCircleRadius());
    EXPECT_DOUBLE_EQ(defense_y_length, field.defenseAreaYLength());
    EXPECT_DOUBLE_EQ(defense_x_length, field.defenseAreaXLength());
    EXPECT_DOUBLE_EQ(9.6, field.totalXLength());
    EXPECT_DOUBLE_EQ(6.6, field.totalYLength());
    EXPECT_DOUBLE_EQ(0.3, field.boundaryMargin());

    EXPECT_EQ(Point(-4.5, 0.0), field.friendlyGoalCenter());
    EXPECT_EQ(Point(4.5, 0.0), field.enemyGoalCenter());

    EXPECT_EQ(Rectangle(Point(-4.68, -0.5), Point(-4.5, 0.5)).getPoints(),
              field.friendlyGoal().getPoints());
    EXPECT_EQ(Rectangle(Point(4.68, -0.5), Point(4.5, 0.5)).getPoints(),
              field.enemyGoal().getPoints());

    EXPECT_EQ(Point(-4.5, 0.5), field.friendlyGoalpostPos());
    EXPECT_EQ(Point(-4.5, -0.5), field.friendlyGoalpostNeg());
    EXPECT_EQ(Point(4.5, 0.5), field.enemyGoalpostPos());
    EXPECT_EQ(Point(4.5, -0.5), field.enemyGoalpostNeg());

    EXPECT_EQ(Rectangle(Point(-4.5, 1.0), Point(-3.5, -1.0)),
              field.friendlyDefenseArea());
    EXPECT_EQ(Rectangle(Point(4.5, 1.0), Point(3.5, -1.0)), field.enemyDefenseArea());
    EXPECT_EQ(Rectangle(Point(-4.5, -3.0), Point(4.5, 3.0)), field.fieldLines());
    EXPECT_EQ(Rectangle(Point(-4.8, -3.3), Point(4.8, 3.3)), field.fieldBoundary());
    EXPECT_EQ(Rectangle(Point(-4.5, -3.0), Point(0, 3.0)), field.friendlyHalf());
    EXPECT_EQ(Rectangle(Point(-4.5, 0), Point(0, 3.0)),
              field.friendlyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(-4.5, 0), Point(0, -3.0)),
              field.friendlyNegativeYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, -3.0), Point(4.5, 3.0)), field.enemyHalf());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(4.5, 3.0)), field.enemyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(4.5, -3.0)), field.enemyNegativeYQuadrant());

    EXPECT_EQ(Point(-3.5, 0.0), field.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field.enemyCornerNeg());

    EXPECT_EQ(Point(0, 0), field.centerPoint());
}

TEST_F(FieldTest, equality_operator_fields_with_different_x_lengths)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    Field field_2 =
        Field(x_length / 2, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_y_lengths)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    Field field_2 =
        Field(x_length, y_length * 2, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_defense_x_length)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    Field field_2 =
        Field(x_length, y_length, defense_x_length * 2, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_defense_y_length)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    Field field_2 =
        Field(x_length, y_length, defense_x_length, defense_y_length / 2, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_goal_x_length)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    Field field_2 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length + 0.2,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_goal_y_length)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    Field field_2 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length + 2, boundary_buffer_size, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_boundary_buffer_size)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    Field field_2 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size * 1.1, center_circle_radius);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_center_circle_radius)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius);

    Field field_2 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_x_length,
              goal_y_length, boundary_buffer_size, center_circle_radius * 10);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, point_not_in_defense_area)
{
    // point around centre
    Point p(2, 3);
    EXPECT_FALSE(field.pointInFriendlyDefenseArea(p));
    EXPECT_FALSE(field.pointInEnemyDefenseArea(p));
}

TEST_F(FieldTest, point_in_friendly_defense_area)
{
    Point p(-4, 0.5);
    EXPECT_TRUE(field.pointInFriendlyDefenseArea(p));
    EXPECT_FALSE(field.pointInEnemyDefenseArea(p));
}

TEST_F(FieldTest, point_in_enemy_defense_area)
{
    Point p(4, -1.0);
    EXPECT_FALSE(field.pointInFriendlyDefenseArea(p));
    EXPECT_TRUE(field.pointInEnemyDefenseArea(p));
}

TEST_F(FieldTest, point_just_outside_enemy_defense_area)
{
    Point p(4, -1.5);
    EXPECT_FALSE(field.pointInFriendlyDefenseArea(p));
    EXPECT_FALSE(field.pointInEnemyDefenseArea(p));
}

TEST_F(FieldTest, point_just_outside_friendly_defense_area)
{
    Point p(-2, -.5);
    EXPECT_FALSE(field.pointInFriendlyDefenseArea(p));
    EXPECT_FALSE(field.pointInEnemyDefenseArea(p));
}

TEST_F(FieldTest, point_in_field_lines)
{
    Point p(4.4, 2.9);
    EXPECT_TRUE(field.pointInFieldLines(p));
}

TEST_F(FieldTest, point_not_in_field_lines)
{
    Point p(4.6, 3.1);
    EXPECT_FALSE(field.pointInFieldLines(p));
}

TEST_F(FieldTest, point_in_entire_field_and_in_field_lines)
{
    Point p(-4.4, 2.9);
    EXPECT_TRUE(field.pointInEntireField(p));
}

TEST_F(FieldTest, point_in_entire_field_and_not_in_field_lines)
{
    Point p(-4.6, 3.22);
    EXPECT_TRUE(field.pointInEntireField(p));
    EXPECT_FALSE(field.pointInFieldLines(p));
}

TEST_F(FieldTest, point_not_in_entire_field)
{
    Point p(-4.91, -0.88);
    EXPECT_FALSE(field.pointInEntireField(p));
}

TEST_F(FieldTest, degenerate_field_zero_lengths)
{
    EXPECT_THROW(Field(0, 2, 3, 1, 0, 0, 0, 4), std::invalid_argument);
    EXPECT_THROW(Field(2, 2, 3, 0, 0, 2, 3, 4), std::invalid_argument);
}

TEST_F(FieldTest, degenerate_field_neg_lengths)
{
    EXPECT_THROW(Field(-4, 2, 3, -1, 0, 5, 5, 4), std::invalid_argument);
    EXPECT_THROW(Field(2, 2, 3, -2, 0, 2, 3, 4), std::invalid_argument);
}
