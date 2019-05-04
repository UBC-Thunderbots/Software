#include "field.h"

#include "boost/circular_buffer.hpp"
#include "geom/rectangle.h"
#include "util/time/timestamp.h"

Field::Field(double field_length, double field_width, double defense_length,
             double defense_width, double goal_width, double boundary_width,
             double center_circle_radius, const Timestamp &timestamp,
             unsigned int buffer_size)
    : field_length_(field_length),
      field_width_(field_width),
      defense_length_(defense_length),
      defense_width_(defense_width),
      goal_width_(goal_width),
      boundary_width_(boundary_width),
      center_circle_radius_(center_circle_radius)
{
    // Set the size of the Timestamp history buffer
    last_update_timestamps.set_capacity(buffer_size);

    updateTimestamp(timestamp);
}

void Field::updateDimensions(const Field &new_field_data)
{
    field_length_          = new_field_data.length();
    field_width_           = new_field_data.width();
    defense_width_         = new_field_data.defenseAreaWidth();
    defense_length_        = new_field_data.defenseAreaLength();
    goal_width_            = new_field_data.goalWidth();
    boundary_width_        = new_field_data.boundaryWidth();
    center_circle_radius_  = new_field_data.centreCircleRadius();
    last_update_timestamps = new_field_data.getTimestampHistory();
}

void Field::updateDimensions(double field_length, double field_width,
                             double defense_length, double defense_width,
                             double goal_width, double boundary_width,
                             double center_circle_radius, const Timestamp &timestamp)
{
    field_length_         = field_length;
    field_width_          = field_width;
    defense_width_        = defense_width;
    defense_length_       = defense_length;
    goal_width_           = goal_width;
    boundary_width_       = boundary_width;
    center_circle_radius_ = center_circle_radius;
    updateTimestamp(timestamp);
}

double Field::length() const
{
    return field_length_;
}

double Field::width() const
{
    return field_width_;
}

double Field::totalLength() const
{
    return field_length_ + (2 * boundary_width_);
}

double Field::totalWidth() const
{
    return field_width_ + (2 * boundary_width_);
}

double Field::goalWidth() const
{
    return goal_width_;
}

double Field::defenseAreaWidth() const
{
    return defense_width_;
}

double Field::defenseAreaLength() const
{
    return defense_length_;
}

Rectangle Field::friendlyDefenseArea() const
{
    return Rectangle(
        Point(-field_length_ * 0.5, defense_width_ / 2.0),
        Point(-field_length_ * 0.5 + defense_length_, -defense_width_ / 2.0));
}

Rectangle Field::enemyDefenseArea() const
{
    return Rectangle(Point(field_length_ * 0.5, defense_width_ / 2.0),
                     Point(field_length_ * 0.5 - defense_length_, -defense_width_ / 2.0));
}

Rectangle Field::fieldLines() const
{
    return Rectangle(friendlyCornerNeg(), enemyCornerPos());
}

double Field::centreCircleRadius() const
{
    return center_circle_radius_;
}

Point Field::centerPoint() const
{
    return Point(0, 0);
}

Point Field::friendlyGoal() const
{
    return Point(-length() / 2.0, 0.0);
}

Point Field::enemyGoal() const
{
    return Point(length() / 2.0, 0.0);
}

Point Field::penaltyEnemy() const
{
    return Point(enemyGoal().x() - defenseAreaLength(), enemyGoal().y());
}

Point Field::penaltyFriendly() const
{
    return Point(friendlyGoal().x() + defenseAreaLength(), friendlyGoal().y());
}

Point Field::friendlyCornerPos() const
{
    return Point(friendlyGoal().x(), width() / 2.0);
}

Point Field::friendlyCornerNeg() const
{
    return Point(friendlyGoal().x(), -width() / 2.0);
}

Point Field::enemyCornerPos() const
{
    return Point(enemyGoal().x(), width() / 2);
}

Point Field::enemyCornerNeg() const
{
    return Point(enemyGoal().x(), -width() / 2);
}

Point Field::friendlyGoalpostPos() const
{
    return Point(friendlyGoal().x(), goalWidth() / 2.0);
}

Point Field::friendlyGoalpostNeg() const
{
    return Point(friendlyGoal().x(), -goalWidth() / 2.0);
}

Point Field::enemyGoalpostPos() const
{
    return Point(enemyGoal().x(), goalWidth() / 2.0);
}

Point Field::enemyGoalpostNeg() const
{
    return Point(enemyGoal().x(), -goalWidth() / 2.0);
}

double Field::boundaryWidth() const
{
    return boundary_width_;
}

bool Field::pointInFriendlyDefenseArea(const Point p) const
{
    return friendlyDefenseArea().containsPoint(p);
}

bool Field::pointInEnemyDefenseArea(const Point p) const
{
    return enemyDefenseArea().containsPoint(p);
}

bool Field::pointInFieldLines(const Point &p) const
{
    return fieldLines().containsPoint(p);
}

boost::circular_buffer<Timestamp> Field::getTimestampHistory() const
{
    return last_update_timestamps;
}

Timestamp Field::getMostRecentTimestamp() const
{
    return last_update_timestamps.front();
}

void Field::updateTimestamp(Timestamp time_stamp) {
    // Check if the timestamp buffer is empty
    if (last_update_timestamps.empty()) {
        last_update_timestamps.push_front(time_stamp);
    }
        // Check that the new timestamp is not older than the most recent timestamp
    else if (time_stamp < Field::getMostRecentTimestamp()) {
        throw std::invalid_argument(
                "Error: Attempt tp update Field state with old Timestamp");
    } else {
        last_update_timestamps.push_front(time_stamp);
    }
}

bool Field::pointInEntireField(const Point &p) const
{
    Rectangle entire_field = Rectangle(Point(-totalLength() / 2, -totalWidth() / 2), Point(totalLength() / 2, totalWidth() / 2));
    return entire_field.containsPoint(p);
}

bool Field::operator==(const Field &other) const
{
    return this->field_width_ == other.field_width_ &&
           this->field_length_ == other.field_length_ &&
           this->goal_width_ == other.goal_width_ &&
           this->defense_width_ == other.defense_width_ &&
           this->defense_length_ == other.defense_length_ &&
           this->boundary_width_ == other.boundary_width_ &&
           this->center_circle_radius_ == other.center_circle_radius_;
}

bool Field::operator!=(const Field &other) const
{
    return !(*this == other);
}
