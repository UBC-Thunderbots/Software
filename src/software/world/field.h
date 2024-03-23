#pragma once

#include "proto/world.pb.h"
#include "software/geom/circle.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/util/make_enum/make_enum.h"

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
     * Creates a field with the standard SSL Division B dimensions
     *
     * @return a field with the standard SSL Division B dimensions
     */
    static Field createSSLDivisionBField();

    /**
     * Creates a field with the standard SSL Division A dimensions
     *
     * @return a field with the standard SSL Division A dimensions
     */
    static Field createSSLDivisionAField();

    /**
     * Creates a field with the standard SSL Division A or B dimensions
     *
     * @param field_type The field type
     * @return a field with the standard SSL Division A or B dimensions
     */
    static Field createField(TbotsProto::FieldType field_type);

    Field() = delete;

    /**
     * Constructs a new field with the given dimensions
     *
     * @pre all dimensions (except for the boundary buffer) must be > 0.
     * @pre the boundary buffer must be >= 0
     *
     * @throws invalid_argument if at least one dimension is <= 0
     *
     * @param field_x_length the length of the playing area (along the x-axis)
     * @param field_y_length the length of the playing area (along the y-axis)
     * @param defense_x_length the length of the defense area (along the x-axis)
     * @param defense_y_length the length of the defense area (along the y-axis)
     * @param goal_x_length the length of the goal (along the x-axis)
     * @param goal_y_length the length of the goal (along the y-axis)
     * @param boundary_buffer_size the size of the boundary area between the edge of the
     * playing area and the physical border/perimeter of the field
     * @param center_circle_radius the radius of the center circle
     */
    explicit Field(double field_x_length, double field_y_length, double defense_x_length,
                   double defense_y_length, double goal_x_length, double goal_y_length,
                   double boundary_buffer_size, double center_circle_radius);

    /**
     * Constructs a new field based on the TbotsProto::Field protobuf representation
     *
     * @pre all dimensions (except for the boundary buffer) must be > 0.
     * @pre the boundary buffer must be >= 0
     *
     * @throws invalid_argument if at least one dimension is <= 0
     *
     * @param field_proto
     */
    explicit Field(const TbotsProto::Field &field_proto);

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
     * Returns the halfway line of the field (as a segment)
     * The segment is on the Y axis and is bounded by the field lines
     *
     * @return the center line of the field
     */
    Segment halfwayLine() const;

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
    const Rectangle &friendlyDefenseArea() const;

    /**
     * Gets the enemy defense area as a Rectangle.
     *
     * @return defense area of the enemy team
     */
    const Rectangle &enemyDefenseArea() const;

    /**
     * Gets the friendly half of the field within field lines
     *
     * @return the friendly half of the field
     */
    Rectangle friendlyHalf() const;

    /**
     * Gets the friendly positive Y quadrant of the field
     *
     * @return the friendly positive Y quadrant of the field
     */
    Rectangle friendlyPositiveYQuadrant() const;

    /**
     * Gets the friendly negative Y quadrant of the field
     *
     * @return the friendly negative Y quadrant of the field
     */
    Rectangle friendlyNegativeYQuadrant() const;

    /**
     * Gets the friendly third of the field within field lines
     *
     * @return the friendly third of the field
     */
    Rectangle friendlyThird() const;

    /**
     * Gets the enemy half of the field within field lines
     *
     * @return the enemy half of the field
     */
    Rectangle enemyHalf() const;

    /**
     * Gets the enemy positive Y quadrant of the field
     *
     * @return the enemy positive Y quadrant of the field
     */
    Rectangle enemyPositiveYQuadrant() const;

    /**
     * Gets the enemy negative Y quadrant of the field
     *
     * @return the enemy negative Y quadrant of the field
     */
    Rectangle enemyNegativeYQuadrant() const;

    /**
     * Gets the enemy third of the field within field lines
     *
     * @return the enemy third of the field
     */
    Rectangle enemyThird() const;

    /**
     * Gets the area within the field lines as a rectangle. This is the set of locations
     * where the ball is considered "in play".
     *
     * @return The area within the field lines as a rectangle
     */
    const Rectangle &fieldLines() const;

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
    Point friendlyGoalCenter() const;

    /**
     * Gets the position of the centre of the enemy goal.
     *
     * @return the position of the enemy goal.
     */
    Point enemyGoalCenter() const;

    /**
     * Gets the area within the friendly goal.
     *
     * @return the area within the friendly goal.
     */
    const Rectangle &friendlyGoal() const;

    /**
     * Gets the area within the enemy goal.
     *
     * @return the area within the enemy goal.
     */
    const Rectangle &enemyGoal() const;

    /**
     * Gets the position of the friendly team's penalty mark.
     *
     * @return the position of the penalty mark for the friendly team.
     */
    Point friendlyPenaltyMark() const;

    /**
     * Gets the position of the enemy team's penalty mark.
     *
     * @return the position of the penalty mark for the enemy team.
     */
    Point enemyPenaltyMark() const;

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
     * @return the size of the margin/bounds around the field in metres
     */
    double boundaryMargin() const;

    /**
     * Returns whether p is in the friendly defense area
     *
     * @returns true if point p is in friendly defense area
     */
    bool pointInFriendlyDefenseArea(const Point &p) const;

    /**
     * Returns whether p is in the enemy defense area
     *
     * @returns true if point p is in enemy defense area
     */
    bool pointInEnemyDefenseArea(const Point &p) const;

    /**
     * Returns true if the point is in the friendly half of the field, and false otherwise
     *
     * @param point
     * @return true if the point is in the friendly half of the field, and false otherwise
     */
    bool pointInFriendlyHalf(const Point &p) const;

    /**
     * Returns true if the point is in the enemy half of the field, and false otherwise
     *
     * @param point
     * @return true if the point is in the enemy's half of the field, and false otherwise
     */
    bool pointInEnemyHalf(const Point &p) const;

    /**
     * Returns true if the point is in the friendly third of the field, and false
     * otherwise
     *
     * @param point
     * @return true if the point is in the friendly third of the field, and false
     * otherwise
     */
    bool pointInFriendlyThird(const Point &p) const;

    /**
     * Returns true if the point is in the enemy third of the field, and false otherwise
     *
     * @param point
     * @return true if the point is in the enemy's third of the field, and false otherwise
     */
    bool pointInEnemyThird(const Point &p) const;

    /**
     * Returns true if the point is in within the provided radius in one of the friendly
     * corner.
     *
     * @param point
     * @param the radius from the corner, to decide whether or not the point is in the
     * field.
     * @return true if the point is in within the provided radius in one of the friendly
     * corner.
     */
    bool pointInFriendlyCorner(const Point &p, double radius) const;

    /**
     * Returns true if the point is in within the provided radius in one of the enemy
     * corner.
     *
     * @param point
     * @param the radius from the corner, to decide whether or not the point is in the
     * field.
     * @return true if the point is in within the provided radius in one of the enemy
     * corner.
     */
    bool pointInEnemyCorner(const Point &p, double radius) const;

    /**
     * Compares two fields for equality
     *
     * @param other the field to compare to
     * @return true if the fields have the same dimensions, and false otherwise
     */
    bool operator==(const Field &other) const;

    /**
     * Compares two fields for inequality
     *
     * @param other the field the compare to
     * @return true if the fields do not have the same dimensions, and false otherwise
     */
    bool operator!=(const Field &other) const;

   private:
    // The length of the playable field (between the goal lines) in metres
    double field_x_length_;
    // The width of the playable field (between the sidelines) in metres
    double field_y_length_;
    // The length of the defense area in metres
    double defense_x_length_;
    // The width of the defense area in metres
    double defense_y_length_;
    // How "deep" the goal is along the x-axis in metres
    double goal_x_length_;
    // The width of the goal (between the goalposts) in metres
    double goal_y_length_;
    // The width of the boundary (between the edge of the marked field lines and the
    // physical border around the field) in metres
    double boundary_buffer_size_;
    // The radius of the center circle in metres
    double center_circle_radius_;
    // The x-coordinate distance from the goal centre to the penalty mark
    double goal_centre_to_penalty_mark_;
    // The following are used for caching to improve performance
    Rectangle enemy_defense_area_;
    Rectangle friendly_defense_area_;
    Rectangle field_lines_;
    Rectangle enemy_goal_;
    Rectangle friendly_goal_;
};

namespace std
{
    // Implements the std::less function so Field can be used as the key in data
    // structures, such as std::map. See:
    // https://stackoverflow.com/questions/42762633/why-is-stdless-better-than and
    // https://en.cppreference.com/w/cpp/utility/functional/less
    template <>
    struct less<Field>
    {
        bool operator()(const Field &lhs, const Field &rhs) const
        {
            return lhs.friendlyDefenseArea().halfPerimeter() +
                       lhs.enemyDefenseArea().halfPerimeter() +
                       lhs.fieldLines().halfPerimeter() +
                       lhs.fieldBoundary().halfPerimeter() <
                   rhs.friendlyDefenseArea().halfPerimeter() +
                       rhs.enemyDefenseArea().halfPerimeter() +
                       rhs.fieldLines().halfPerimeter() +
                       rhs.fieldBoundary().halfPerimeter();
        }
    };
}  // namespace std
