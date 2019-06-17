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
                      boundary_width, center_circle_radius, default_time_stamp);
    }

    Field field = Field(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0));
    double length;
    double width;
    double defense_length;
    double defense_width;
    double goal_width;
    double boundary_width;
    double center_circle_radius;
    Timestamp default_time_stamp = Timestamp::fromSeconds(0);
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
    EXPECT_EQ(default_time_stamp, field.getMostRecentTimestamp());
}

TEST_F(FieldTest, update_with_all_parameters)
{
    Field field_to_update = Field(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0));

    field_to_update.updateDimensions(length, width, defense_length, defense_width,
                                     goal_width, boundary_width, center_circle_radius,
                                     default_time_stamp);

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
    EXPECT_EQ(Rectangle(Point(-4.5, -3.0), Point(4.5, 3.0)),
              field_to_update.fieldLines());

    EXPECT_EQ(Point(-3.5, 0.0), field_to_update.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field_to_update.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field_to_update.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field_to_update.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field_to_update.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field_to_update.enemyCornerNeg());

    EXPECT_EQ(Point(0, 0), field_to_update.centerPoint());
    EXPECT_EQ(default_time_stamp, field_to_update.getMostRecentTimestamp());
}

TEST_F(FieldTest, update_with_new_field)
{
    Field field_to_update = Field(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0));

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
    EXPECT_EQ(Rectangle(Point(-4.5, -3.0), Point(4.5, 3.0)),
              field_to_update.fieldLines());

    EXPECT_EQ(Point(-3.5, 0.0), field_to_update.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field_to_update.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field_to_update.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field_to_update.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field_to_update.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field_to_update.enemyCornerNeg());

    EXPECT_EQ(Point(0, 0), field_to_update.centerPoint());
    EXPECT_EQ(default_time_stamp, field_to_update.getMostRecentTimestamp());
}

TEST_F(FieldTest, equality_operator_fields_with_different_lengths)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    Field field_2 = Field(length / 2, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}
TEST_F(FieldTest, equality_operator_fields_with_different_widths)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    Field field_2 = Field(length, width * 2, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_defense_length)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    Field field_2 = Field(length, width, defense_length * 2, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_defense_width)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    Field field_2 = Field(length, width, defense_length, defense_width / 2, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_goal_width)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    Field field_2 = Field(length, width, defense_length, defense_width, 0, boundary_width,
                          center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_boundary_width)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    Field field_2 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width * 1.1, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_center_circle_radius)
{
    Field field_1 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius, default_time_stamp);

    Field field_2 = Field(length, width, defense_length, defense_width, goal_width,
                          boundary_width, center_circle_radius * 10, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

// Test that the timestamp history is saved when the Field is updated
TEST_F(FieldTest, field_timestamp_history_is_saved)
{
    Field field = Field(length, width, defense_length, defense_width, goal_width,
                        boundary_width, center_circle_radius, default_time_stamp);

    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius,
                           Timestamp::fromSeconds(default_time_stamp.getSeconds() + 1));

    field.updateDimensions(length, width, defense_length, defense_width, goal_width,
                           boundary_width, center_circle_radius,
                           Timestamp::fromSeconds(default_time_stamp.getSeconds() + 2));

    EXPECT_EQ(field.getTimestampHistory().size(), 3);
    EXPECT_EQ(field.getTimestampHistory()[0].getSeconds(),
              default_time_stamp.getSeconds() + 2);
    EXPECT_EQ(field.getTimestampHistory()[1].getSeconds(),
              default_time_stamp.getSeconds() + 1);
    EXPECT_EQ(field.getTimestampHistory()[2].getSeconds(),
              default_time_stamp.getSeconds());

    EXPECT_EQ(field.getMostRecentTimestamp().getSeconds(),
              default_time_stamp.getSeconds() + 2);
}

TEST_F(FieldTest, exception_thrown_when_older_timestamp_is_used)
{
    ASSERT_THROW(
        Field field = Field(length, width, defense_length, defense_width, goal_width,
                            boundary_width, center_circle_radius,
                            Timestamp::fromSeconds(default_time_stamp.getSeconds() - 1)),
        std::invalid_argument);
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
