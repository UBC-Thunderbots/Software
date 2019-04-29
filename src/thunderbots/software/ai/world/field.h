#pragma once

#include "geom/point.h"
#include "geom/rectangle.h"

typedef enum
{
    EAST,  // positive X side according to vision
    WEST   // negative X side
} FieldSide;

/**
 * Exposes the dimensions of various parts of the field.
 */
class Field
{
   public:
    /**
     * Constructs a new field with the given dimensions
     *
     * @param field_length the length of the playing area (along the x-axis)
     * @param field_width the width of the playing area (along the y-axis)
     * @param defense_length the length of the defense area (along the x-axis)
     * @param defense_width the width of the defense area (along the y-axis)
     * @param goal_width the width of the goal (along the y-axis)
     * @param boundary_width the width/size of the boundary area between the edge of the
     * playing area and the physical border/perimeter of the field
     * @param center_circle_radius the radius of the center circle
     */
    explicit Field(double field_length, double field_width, double defense_length,
                   double defense_width, double goal_width, double boundary_width,
                   double center_circle_radius);

    /**
     * Updates the dimensions of the field. All units should be in metres.
     *
     * @param field_length the length of the playing area (along the x-axis)
     * @param field_width the width of the playing area (along the y-axis)
     * @param defense_length the length of the defense area (along the x-axis)
     * @param defense_width the width of the defense area (along the y-axis)
     * @param goal_width the width of the goal (along the y-axis)
     * @param boundary_width the width/size of the boundary area between the edge of the
     * playing area and the physical border/perimeter of the field
     * @param center_circle_radius the radius of the center circle
     */
    void updateDimensions(double field_length, double field_width, double defense_length,
                          double defense_width, double goal_width, double boundary_width,
                          double center_circle_radius);

    /**
     * Updates the field with new data
     *
     * @param new_ball_data A field containing new field data
     */
    void updateDimensions(const Field &new_field_data);

    /**
     * Gets the length of the field from goal-line to goal-line in metres.
     *
     * @return the length of the field in metres.
     */
    double length() const;

    /**
     * Gets the length of the field including the boundary area in metres.
     *
     * @return the total length of the field in metres, including the boundary area.
     */
    double totalLength() const;

    /**
     * Gets the width of the field from sideline to sideline in metres.
     *
     * @return the width of the field in metres.
     */
    double width() const;

    /**
     * Gets the width of the field including the boundary area in metres.
     *
     * @return the total width of the field in metres, including the boundary area.
     */
    double totalWidth() const;

    /**
     * Gets the width of the goal, symmetric above and below the centreline,
     * from goalpost to goalpost in metres.
     *
     * @return the width of the goal in metres.
     */
    double goalWidth() const;

    /**
     * Gets the radius of the centre circle in metres.
     *
     * @return the radius of the centre circle in metres.
     */
    double centreCircleRadius() const;

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
    double defenseAreaWidth() const;

    /**
     * Gets the length of the defense area in metres, which runs along the x-axis. This
     * is how far the defense area extends in front of the goal.
     *
     * @return the width of the straight parts in metres.
     */
    double defenseAreaLength() const;

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
     * Gets the area within the field lines as a rectangle. This is the set of locations
     * where the ball is considered "in play".
     *
     * @return The area within the field lines as a rectangle
     */
    Rectangle fieldLines() const;

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
    double boundaryWidth() const;

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
     * @param p
     *
     * @return true if p is within the field lines of the field, false otherwise
     */
    bool pointInFieldLines(const Point &p) const;

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
    // Private variables have underscores at the end of their names
    // to avoid conflicts with function names

    // The length of the playable field (between the goal lines) in metres
    double field_length_;
    // The width of the playable field (between the sidelines) in metres
    double field_width_;
    // The width of the goal (between the goalposts) in metres
    double goal_width_;
    // The width of the defense area in metres
    double defense_width_;
    // The length of the defense area in metres
    double defense_length_;
    // The width of the boundary (between the edge of the marked field lines and the
    // physical border around the field) in metres
    double boundary_width_;
    // The radius of the center circle in metres
    double center_circle_radius_;
};
