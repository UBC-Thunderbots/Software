#include "software/world/field.h"

#include <boost/circular_buffer.hpp>

#include "shared/constants.h"
#include "software/new_geom/rectangle.h"
#include "software/time/timestamp.h"

Field::Field(double field_x_length, double field_y_length, double defense_x_length,
             double defense_y_length, double goal_y_length, double boundary_buffer_size,
             double center_circle_radius, const Timestamp &timestamp,
             unsigned int buffer_size)
    : field_x_length_(field_x_length),
      field_y_length_(field_y_length),
      defense_x_length_(defense_x_length),
      defense_y_length_(defense_y_length),
      goal_y_length_(goal_y_length),
      // While not explicitly given by SSL-Vision, the goals are typically
      // deep enough to fit a single robot
      goal_x_length_(ROBOT_MAX_RADIUS_METERS * 2),
      boundary_buffer_size_(boundary_buffer_size),
      center_circle_radius_(center_circle_radius)
{
    // Set the size of the Timestamp history buffer
    last_update_timestamps.set_capacity(buffer_size);

    updateTimestamp(timestamp);
}

void Field::updateDimensions(const Field &new_field_data)
{
    field_x_length_        = new_field_data.xLength();
    field_y_length_        = new_field_data.yLength();
    defense_y_length_      = new_field_data.defenseAreaYLength();
    defense_x_length_      = new_field_data.defenseAreaXLength();
    goal_y_length_         = new_field_data.goalYLength();
    boundary_buffer_size_  = new_field_data.boundaryYLength();
    center_circle_radius_  = new_field_data.centerCircleRadius();
    last_update_timestamps = new_field_data.getTimestampHistory();
}

void Field::updateDimensions(double field_x_length, double field_y_length,
                             double defense_x_length, double defense_y_length,
                             double goal_y_length, double boundary_buffer_size,
                             double center_circle_radius, const Timestamp &timestamp)
{
    field_x_length_       = field_x_length;
    field_y_length_       = field_y_length;
    defense_y_length_     = defense_y_length;
    defense_x_length_     = defense_x_length;
    goal_y_length_        = goal_y_length;
    boundary_buffer_size_ = boundary_buffer_size;
    center_circle_radius_ = center_circle_radius;
    updateTimestamp(timestamp);
}

double Field::xLength() const
{
    return field_x_length_;
}

double Field::yLength() const
{
    return field_y_length_;
}

double Field::totalXLength() const
{
    return field_x_length_ + (2 * boundary_buffer_size_);
}

double Field::totalYLength() const
{
    return field_y_length_ + (2 * boundary_buffer_size_);
}

double Field::goalYLength() const
{
    return goal_y_length_;
}

double Field::goalXLength() const
{
    return goal_x_length_;
}

double Field::defenseAreaYLength() const
{
    return defense_y_length_;
}

double Field::defenseAreaXLength() const
{
    return defense_x_length_;
}

Rectangle Field::friendlyDefenseArea() const
{
    return Rectangle(
        Point(-field_x_length_ * 0.5, defense_y_length_ / 2.0),
        Point(-field_x_length_ * 0.5 + defense_x_length_, -defense_y_length_ / 2.0));
}

Rectangle Field::enemyDefenseArea() const
{
    return Rectangle(
        Point(field_x_length_ * 0.5, defense_y_length_ / 2.0),
        Point(field_x_length_ * 0.5 - defense_x_length_, -defense_y_length_ / 2.0));
}

Rectangle Field::friendlyHalf() const
{
    return Rectangle(friendlyCornerNeg(), Point(0, friendlyCornerPos().y()));
}

Rectangle Field::friendlyPositiveYQuadrant() const
{
    return Rectangle(friendlyGoal(), Point(0, friendlyCornerPos().y()));
}

Rectangle Field::friendlyNegativeYQuadrant() const
{
    return Rectangle(friendlyGoal(), Point(0, friendlyCornerNeg().y()));
}

Rectangle Field::enemyHalf() const
{
    return Rectangle(Point(0, enemyCornerNeg().y()), enemyCornerPos());
}

Rectangle Field::enemyPositiveYQuadrant() const
{
    return Rectangle(centerPoint(), enemyCornerPos());
}

Rectangle Field::enemyNegativeYQuadrant() const
{
    return Rectangle(centerPoint(), enemyCornerNeg());
}

Rectangle Field::fieldLines() const
{
    return Rectangle(friendlyCornerNeg(), enemyCornerPos());
}

Rectangle Field::fieldBoundary() const
{
    Point neg_x_neg_y_corner(-totalXLength() / 2, -totalYLength() / 2);
    Point pos_x_pos_y_corner(totalXLength() / 2, totalYLength() / 2);
    return Rectangle(neg_x_neg_y_corner, pos_x_pos_y_corner);
}

bool Field::isValid() const
{
    if (totalXLength() < GeomConstants::FIXED_EPSILON ||
        totalYLength() < GeomConstants::FIXED_EPSILON)
    {
        return false;
    }
    return true;
}

double Field::centerCircleRadius() const
{
    return center_circle_radius_;
}

Circle Field::centerCircle() const
{
    return Circle(Point(0, 0), centerCircleRadius());
}

Point Field::centerPoint() const
{
    return Point(0, 0);
}

Point Field::friendlyGoal() const
{
    return Point(-xLength() / 2.0, 0.0);
}

Point Field::enemyGoal() const
{
    return Point(xLength() / 2.0, 0.0);
}

Point Field::penaltyEnemy() const
{
    return Point(enemyGoal().x() - defenseAreaXLength(), enemyGoal().y());
}

Point Field::penaltyFriendly() const
{
    return Point(friendlyGoal().x() + defenseAreaXLength(), friendlyGoal().y());
}

Point Field::friendlyCornerPos() const
{
    return Point(friendlyGoal().x(), yLength() / 2.0);
}

Point Field::friendlyCornerNeg() const
{
    return Point(friendlyGoal().x(), -yLength() / 2.0);
}

Point Field::enemyCornerPos() const
{
    return Point(enemyGoal().x(), yLength() / 2);
}

Point Field::enemyCornerNeg() const
{
    return Point(enemyGoal().x(), -yLength() / 2);
}

Point Field::friendlyGoalpostPos() const
{
    return Point(friendlyGoal().x(), goalYLength() / 2.0);
}

Point Field::friendlyGoalpostNeg() const
{
    return Point(friendlyGoal().x(), -goalYLength() / 2.0);
}

Point Field::enemyGoalpostPos() const
{
    return Point(enemyGoal().x(), goalYLength() / 2.0);
}

Point Field::enemyGoalpostNeg() const
{
    return Point(enemyGoal().x(), -goalYLength() / 2.0);
}

double Field::boundaryYLength() const
{
    return boundary_buffer_size_;
}

bool Field::pointInFriendlyDefenseArea(const Point p) const
{
    return friendlyDefenseArea().contains(p);
}

bool Field::pointInEnemyDefenseArea(const Point p) const
{
    return enemyDefenseArea().contains(p);
}

bool Field::pointInFieldLines(const Point &p) const
{
    return fieldLines().contains(p);
}

boost::circular_buffer<Timestamp> Field::getTimestampHistory() const
{
    return last_update_timestamps;
}

Timestamp Field::getMostRecentTimestamp() const
{
    return last_update_timestamps.front();
}

void Field::updateTimestamp(Timestamp time_stamp)
{
    // Check if the timestamp buffer is empty
    if (last_update_timestamps.empty())
    {
        last_update_timestamps.push_front(time_stamp);
    }
    // Check that the new timestamp is not older than the most recent timestamp
    else if (time_stamp < Field::getMostRecentTimestamp())
    {
        throw std::invalid_argument(
            "Error: Attempt tp update Field state with old Timestamp");
    }
    else
    {
        last_update_timestamps.push_front(time_stamp);
    }
}

bool Field::pointInEntireField(const Point &p) const
{
    Rectangle entire_field = Rectangle(Point(-totalXLength() / 2, -totalYLength() / 2),
                                       Point(totalXLength() / 2, totalYLength() / 2));
    return entire_field.contains(p);
}

bool Field::operator==(const Field &other) const
{
    return this->field_y_length_ == other.field_y_length_ &&
           this->field_x_length_ == other.field_x_length_ &&
           this->goal_y_length_ == other.goal_y_length_ &&
           this->defense_y_length_ == other.defense_y_length_ &&
           this->defense_x_length_ == other.defense_x_length_ &&
           this->boundary_buffer_size_ == other.boundary_buffer_size_ &&
           this->center_circle_radius_ == other.center_circle_radius_;
}

bool Field::operator!=(const Field &other) const
{
    return !(*this == other);
}
