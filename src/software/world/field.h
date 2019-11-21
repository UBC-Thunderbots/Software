#pragma once

#include "boost/circular_buffer.hpp"
#include "software/geom/circle.h"
#include "software/geom/rectangle.h"
#include "software/new_geom/point.h"
#include "software/util/time/timestamp.h"

typedef enum
{
    POS_X,  // positive X side according to vision
    NEG_X   // negative X side
} FieldSide;

/**
 * Exposes the dimensions of various parts of the field.
 *
 *      +-------+-------+-------+
 *      |       | Enemy |       |
 *      |       +-------+       |
 *      |                       |
 *      |           ^ +x        |
 *      |           |           |
 *      |           |           |
 *      |           |           |
 *      |  +y       |           |
 *      |  <--------+ (0,0)     |
 *      |                       |
 *      |                       |
 *      |                       |
 *      |      +----------+     |
 *      |      | Friendly |     |
 *      +------+----------+-----+
 */
class Field
{
   public:
    /**
     * Constructs a new field with the given dimensions
     *
     * @param field_x_length the length of the playing area (along the x-axis)
     * @param field_y_length the length of the playing area (along the y-axis)
     * @param defense_x_length the length of the defense area (along the x-axis)
     * @param defense_y_length the length of the defense area (along the y-axis)
     * @param goal_y_length the length of the goal (along the y-axis)
     * @param boundary_buffer_size the size of the boundary area between the edge of the
     * playing area and the physical border/perimeter of the field
     * @param center_circle_radius the radius of the center circle
     * @param timestamp the Timestamp associated with the creation of the Field object
     */
    explicit Field(double field_x_length, double field_y_length, double defense_x_length,
                   double defense_y_length, double goal_y_length,
                   double boundary_buffer_size, double center_circle_radius,
                   const Timestamp &timestamp, unsigned int buffer_size = 20);

    /**
     * Updates the dimensions of the field. All units should be in metres.
     *
     * @param field_x_length the length of the playing area (along the x-axis)
     * @param field_y_length the length of the playing area (along the y-axis)
     * @param defense_x_length the length of the defense area (along the x-axis)
     * @param defense_y_length the length of the defense area (along the y-axis)
     * @param goal_y_length the length of the goal (along the y-axis)
     * @param boundary_buffer_size the size of the boundary area between the edge of thet
     * playing area and the physical border/perimeter of the field
     * @param center_circle_radius the radius of the center circle
     * @param timestamp the Timestamp corresponding to any updates to the Field object
     */
    void updateDimensions(double field_x_length, double field_y_length,
                          double defense_x_length, double defense_y_length,
                          double goal_y_length, double boundary_buffer_size,
                          double center_circle_radius, const Timestamp &timestamp);

    /**
     * Updates the field with new data
     *
     * @param new_ball_data A field containing new field data
     */
    void updateDimensions(const Field &new_field_data);

    /**
     * Gets the x-axis length of the field from goal-line to goal-line in metres.
     *
     * @return the x-axis length of the field in metres.
     */
    double xLength() const;

    /**
     * Gets the x-axis length of the field including the boundary area in metres.
     *
     * @return the total x-axis length of the field in metres, including the boundary
     * area.
     */
    double totalXLength() const;

    /**
     * Gets the y-axis length of the field from sideline to sideline in metres.
     *
     * @return the y-axis length of the field in metres.
     */
    double yLength() const;

    /**
     * Gets the y-axis length of the field including the boundary area in metres.
     *
     * @return the total y-axis length of the field in metres, including the boundary
     * area.
     */
    double totalYLength() const;

    /**
     * Gets the y-axis length of the goal, symmetric above and below the centreline,
     * from goalpost to goalpost in metres.
     *
     * @return the y-axis length of the goal in metres.
     */
    double goalYLength() const;

    /**
     * Gets the "depth" of the goal along the x-axis, in metres.
     *
     * @return the "depth" of the goal along the x-axis, in metres.
     */
    double goalXLength() const;

    /**
     * Gets the radius of the centre circle in metres.
     *
     * @return the radius of the centre circle in metres.
     */
    double centerCircleRadius() const;

    /**
     * Returns the center circle at the middle of the field
     *
     * @return the center circle at the middle of the field
     */
    Circle centerCircle() const;

    /**
     * Returns the center point of the field
     *
     * @return the center point of the field
     */
    Point centerPoint() const;

    /**
     * Gets the width of the defense area in metres, which runs along the y-axis. This is
     * the total width of how far the defense area stretches from one side of the goal to
     * the other.
     *
     * @return the width of the defense area
     */
    double defenseAreaYLength() const;

    /**
     * Gets the length of the defense area in metres, which runs along the x-axis. This
     * is how far the defense area extends in front of the goal.
     *
     * @return the width of the straight parts in metres.
     */
    double defenseAreaXLength() const;

    /**
     * Gets the friendly defense area as a Rectangle.
     *
     * @return defense area of the friendly team
     */
    Rectangle friendlyDefenseArea() const;

    /**
     * Gets the enemy defense area as a Rectangle.
     *
     * @return defense area of the enemy team
     */
    Rectangle enemyDefenseArea() const;

    /**
     * Gets the friendly half of the field
     *
     * @return the friendly half of the field
     */
    Rectangle friendlyHalf() const;

    /**
     * Gets the enemy half of the field
     *
     * @return the enemy half of the field
     */
    Rectangle enemyHalf() const;

    /**
     * Gets the area within the field lines as a rectangle. This is the set of locations
     * where the ball is considered "in play".
     *
     * @return The area within the field lines as a rectangle
     */
    Rectangle fieldLines() const;

    /**
     * Gets the area within the field boundary (the physical walls surrounding the Field).
     * This is the entire area where the robots and ball can move, and is a superset of
     * the area inside the field lines.
     *
     * @return The area within the field boundary as a rectangle
     */
    Rectangle fieldBoundary() const;

    /**
     * Gets the position of the centre of the friendly goal.
     *
     * @return the position of the friendly goal.
     */
    Point friendlyGoal() const;

    /**
     * Gets the position of the centre of the enemy goal.
     *
     * @return the position of the enemy goal.
     */
    Point enemyGoal() const;

    /**
     * Gets the position of the penalty mark near the enemy goal.
     *
     * @return the position of the penalty mark near the enemy goal
     */
    Point penaltyEnemy() const;

    /**
     * Gets the position of the penalty mark near the friendly goal.
     *
     * @return the position of the penalty mark near the friendly goal
     */
    Point penaltyFriendly() const;

    /**
     * Gets the position of our corner with the positive y-axis
     *
     * @return the position of our corner with the positive y-axis
     */
    Point friendlyCornerPos() const;

    /**
     * Gets the position of our corner with the negative y-axis
     *
     * @return the position of our corner with the negative y-axis
     */
    Point friendlyCornerNeg() const;

    /**
     * Gets the position of the enemy corner with the positive y-axis
     *
     * @return the position of the enemy corner with the positive y-axis
     */
    Point enemyCornerPos() const;

    /**
     * Gets the position of the enemy corner with the negative y-axis
     *
     * @return the position of the enemy corner with the negative y-axis
     */
    Point enemyCornerNeg() const;

    /**
     * Gets the position of the friendly goalpost with the positive y-axis
     *
     * @return the position of the friendly goalpost with the positive y-axis
     */
    Point friendlyGoalpostPos() const;

    /**
     * Gets the position of the friendly goalpost with the negative y-axis
     *
     * @return the position of the friendly goalpost with the negative y-axis
     */
    Point friendlyGoalpostNeg() const;

    /**
     * Gets the position of the enemy goalpost with the positive y-axis
     *
     * @return the position of the enemy goalpost with the positive y-axis
     */
    Point enemyGoalpostPos() const;

    /**
     * Gets the position of the enemy goalpost with the negative y-axis
     *
     * @return the position of the enemy goalpost with the negative y-axis
     */
    Point enemyGoalpostNeg() const;

    /**
     * Gets the margin for being out of bounds on the top or bottom of the
     * field in metres.
     *
     * @return the size of the margin/bounds around the field
     */
    double boundaryYLength() const;

    /**
     * Returns whether p is in the friendly defense area
     *
     * @returns true if point p is in friendly defense area
     */
    bool pointInFriendlyDefenseArea(const Point p) const;

    /**
     * Returns whether p is in the enemy defense area
     *
     * @returns true if point p is in enemy defense area
     */
    bool pointInEnemyDefenseArea(const Point p) const;

    /**
     * Returns whether p is within the field lines of the this field.
     *
     * @param p The point to check
     *
     * @return true if p is within the field lines of the field, false otherwise
     */
    bool pointInFieldLines(const Point &p) const;

    /**
     * Returns whether p is anywhere within the entire field, including the boundary
     * area around the field lines. ie, if the point is within the total width and height
     * of the field.
     *
     * @param p The point to check
     *
     * @return true if p is anywhere within the entire field, and false otherwise
     */
    bool pointInEntireField(const Point &p) const;

    /**
     * Compares two fields for equality
     *
     * @param other the field to compare to
     * @return true if the fields have the same dimensions, and false otherwise
     */
    bool operator==(const Field &other) const;

    /**
     * Returns the entire update Timestamp history for Field object
     *
     * @return boost::circular_buffer of Timestamp history for the Field object
     */
    boost::circular_buffer<Timestamp> getTimestampHistory() const;

    /**
     * Returns the most Timestamp corresponding to the most recent update to Field object
     *
     * @return Timestamp : The Timestamp corresponding to the most recent update to the
     * Field object
     */
    Timestamp getMostRecentTimestamp() const;

    /**
     * Compares two fields for inequality
     *
     * @param other the field the compare to
     * @return true if the fields do not have the same dimensions, and false otherwise
     */
    bool operator!=(const Field &other) const;

    /**
     * Updates the timestamp history for the Field object
     *
     * @param time_stamp : The timestamp at which the Field object was updated
     */
    void updateTimestamp(Timestamp time_stamp);

   private:
    // Private variables have underscores at the end of their names
    // to avoid conflicts with function names

    // The length of the playable field (between the goal lines) in metres
    double field_x_length_;
    // The width of the playable field (between the sidelines) in metres
    double field_y_length_;
    // The length of the defense area in metres
    double defense_x_length_;
    // The width of the defense area in metres
    double defense_y_length_;
    // The width of the goal (between the goalposts) in metres
    double goal_y_length_;
    // How "deep" the goal is along the x-axis in metres
    double goal_x_length_;

    // The width of the boundary (between the edge of the marked field lines and the
    // physical border around the field) in metres
    double boundary_buffer_size_;
    // The radius of the center circle in metres
    double center_circle_radius_;
    // All previous timestamps of when the field was updated, with the most recent
    // timestamp at the front of the queue,
    boost::circular_buffer<Timestamp> last_update_timestamps;
};
