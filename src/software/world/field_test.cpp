#include "software/world/field.h"

#include <gtest/gtest.h>

#include "shared/constants.h"

class FieldTest : public ::testing::Test
{
   protected:
    void SetUp() override
    {
        x_length             = 9.0;
        y_length             = 6.0;
        defense_x_length     = 1.0;
        defense_y_length     = 2.0;
        goal_y_length        = 1.0;
        boundary_buffer_size = 0.3;
        center_circle_radius = 0.5;

        field =
            Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
                  boundary_buffer_size, center_circle_radius, default_time_stamp);
    }

    Field field = Field(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0));
    double x_length;
    double y_length;
    double defense_x_length;
    double defense_y_length;
    double goal_y_length;
    double boundary_buffer_size;
    double center_circle_radius;
    Timestamp default_time_stamp = Timestamp::fromSeconds(0);
};

TEST_F(FieldTest, construct_with_parameters)
{
    // The field was already constructed in the test setup, so we only need to check
    // values here
    EXPECT_DOUBLE_EQ(x_length, field.xLength());
    EXPECT_DOUBLE_EQ(y_length, field.yLength());
    EXPECT_DOUBLE_EQ(goal_y_length, field.goalYLength());
    EXPECT_DOUBLE_EQ(ROBOT_MAX_RADIUS_METERS * 2, field.goalXLength());
    EXPECT_DOUBLE_EQ(center_circle_radius, field.centerCircleRadius());
    EXPECT_DOUBLE_EQ(defense_y_length, field.defenseAreaYLength());
    EXPECT_DOUBLE_EQ(defense_x_length, field.defenseAreaXLength());
    EXPECT_EQ(default_time_stamp, field.getMostRecentTimestamp());
}

TEST_F(FieldTest, update_with_all_parameters)
{
    Field field_to_update = Field(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0));

    field_to_update.updateDimensions(
        x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
        boundary_buffer_size, center_circle_radius, default_time_stamp);

    EXPECT_DOUBLE_EQ(9.6, field_to_update.totalXLength());
    EXPECT_DOUBLE_EQ(6.6, field_to_update.totalYLength());
    EXPECT_DOUBLE_EQ(0.3, field_to_update.boundaryYLength());

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
    EXPECT_EQ(Rectangle(Point(-4.8, -3.3), Point(4.8, 3.3)),
              field_to_update.fieldBoundary());
    EXPECT_EQ(Rectangle(Point(-4.5, -3.0), Point(0, 3.0)),
              field_to_update.friendlyHalf());
    EXPECT_EQ(Rectangle(Point(-4.5, 0), Point(0, 3.0)),
              field_to_update.friendlyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(-4.5, 0), Point(0, -3.0)),
              field_to_update.friendlyNegativeYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, -3.0), Point(4.5, 3.0)), field_to_update.enemyHalf());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(4.5, 3.0)),
              field_to_update.enemyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(4.5, -3.0)),
              field_to_update.enemyNegativeYQuadrant());

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

    EXPECT_DOUBLE_EQ(9.6, field_to_update.totalXLength());
    EXPECT_DOUBLE_EQ(6.6, field_to_update.totalYLength());
    EXPECT_DOUBLE_EQ(0.3, field_to_update.boundaryYLength());

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
    EXPECT_EQ(Rectangle(Point(-4.8, -3.3), Point(4.8, 3.3)),
              field_to_update.fieldBoundary());
    EXPECT_EQ(Rectangle(Point(-4.5, -3.0), Point(0, 3.0)),
              field_to_update.friendlyHalf());
    EXPECT_EQ(Rectangle(Point(-4.5, 0), Point(0, 3.0)),
              field_to_update.friendlyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(-4.5, 0), Point(0, -3.0)),
              field_to_update.friendlyNegativeYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, -3.0), Point(4.5, 3.0)), field_to_update.enemyHalf());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(4.5, 3.0)),
              field_to_update.enemyPositiveYQuadrant());
    EXPECT_EQ(Rectangle(Point(0, 0), Point(4.5, -3.0)),
              field_to_update.enemyNegativeYQuadrant());

    EXPECT_EQ(Point(-3.5, 0.0), field_to_update.penaltyFriendly());
    EXPECT_EQ(Point(3.5, 0.0), field_to_update.penaltyEnemy());

    EXPECT_EQ(Point(-4.5, 3.0), field_to_update.friendlyCornerPos());
    EXPECT_EQ(Point(-4.5, -3.0), field_to_update.friendlyCornerNeg());
    EXPECT_EQ(Point(4.5, 3.0), field_to_update.enemyCornerPos());
    EXPECT_EQ(Point(4.5, -3.0), field_to_update.enemyCornerNeg());

    EXPECT_EQ(Point(0, 0), field_to_update.centerPoint());
    EXPECT_EQ(default_time_stamp, field_to_update.getMostRecentTimestamp());
}

TEST_F(FieldTest, equality_operator_fields_with_different_x_lengths)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    Field field_2 =
        Field(x_length / 2, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}
TEST_F(FieldTest, equality_operator_fields_with_different_y_lengths)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    Field field_2 =
        Field(x_length, y_length * 2, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_defense_x_length)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    Field field_2 =
        Field(x_length, y_length, defense_x_length * 2, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_defense_y_length)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    Field field_2 =
        Field(x_length, y_length, defense_x_length, defense_y_length / 2, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_goal_y_length)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    Field field_2 = Field(x_length, y_length, defense_x_length, defense_y_length, 0,
                          boundary_buffer_size, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_boundary_buffer_size)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    Field field_2 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size * 1.1, center_circle_radius, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

TEST_F(FieldTest, equality_operator_fields_with_different_center_circle_radius)
{
    Field field_1 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    Field field_2 =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius * 10, default_time_stamp);

    EXPECT_NE(field_1, field_2);
}

// Test that the timestamp history is saved when the Field is updated
TEST_F(FieldTest, field_timestamp_history_is_saved)
{
    Field field =
        Field(x_length, y_length, defense_x_length, defense_y_length, goal_y_length,
              boundary_buffer_size, center_circle_radius, default_time_stamp);

    field.updateDimensions(x_length, y_length, defense_x_length, defense_y_length,
                           goal_y_length, boundary_buffer_size, center_circle_radius,
                           Timestamp::fromSeconds(default_time_stamp.getSeconds() + 1));

    field.updateDimensions(x_length, y_length, defense_x_length, defense_y_length,
                           goal_y_length, boundary_buffer_size, center_circle_radius,
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
        Field field = Field(x_length, y_length, defense_x_length, defense_y_length,
                            goal_y_length, boundary_buffer_size, center_circle_radius,
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

TEST_F(FieldTest, update_timestamp_with_timestamp_in_future)
{
    try
    {
        field.updateTimestamp(Timestamp::fromSeconds(1.0));
        EXPECT_EQ(Timestamp::fromSeconds(1.0), field.getMostRecentTimestamp());
    }
    catch (std::invalid_argument&)
    {
        ADD_FAILURE() << "Invalid Timestamp used to update field timestamp";
    }
}

TEST_F(FieldTest, update_timestamp_with_same_timestamp)
{
    try
    {
        field.updateTimestamp(Timestamp::fromSeconds(0.0));
        EXPECT_EQ(Timestamp::fromSeconds(0.0), field.getMostRecentTimestamp());
    }
    catch (std::invalid_argument&)
    {
        ADD_FAILURE() << "Invalid Timestamp used to update field timestamp";
    }
}

TEST_F(FieldTest, update_timestamp_with_timestamp_in_past)
{
    // First make sure we have a timestamp > 0
    field.updateTimestamp(Timestamp::fromSeconds(1.0));
    EXPECT_THROW(field.updateTimestamp(Timestamp::fromSeconds(0.9)),
                 std::invalid_argument);
}
