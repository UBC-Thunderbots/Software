#include "ai/world/field.h"

Field::Field()
    : valid_(false),
      field_length_(0),
      field_width_(0),
      goal_width_(0),
      defense_length_(0),
      defense_width_(0),
      boundary_width_(0),
      center_circle_radius_(0)
{
}

void Field::updateDimensions(thunderbots_msgs::Field new_field_msg)
{
    valid_                = true;
    field_length_         = new_field_msg.field_length;
    field_width_          = new_field_msg.field_width;
    goal_width_           = new_field_msg.goal_width;
    defense_width_        = new_field_msg.defense_width;
    defense_length_       = new_field_msg.defense_length;
    boundary_width_       = new_field_msg.boundary_width;
    center_circle_radius_ = new_field_msg.center_circle_radius;
}

void Field::updateDimensions(const Field &new_field_data)
{
    valid_                = true;
    field_length_         = new_field_data.length();
    field_width_          = new_field_data.width();
    goal_width_           = new_field_data.goalWidth();
    defense_width_        = new_field_data.defenseAreaWidth();
    defense_length_       = new_field_data.defenseAreaLength();
    boundary_width_       = new_field_data.boundaryWidth();
    center_circle_radius_ = new_field_data.centreCircleRadius();
}

void Field::updateDimensions(double field_length, double field_width, double goal_width,
                             double defense_length, double defense_width,
                             double boundary_width, double center_circle_radius)
{
    valid_                = true;
    field_length_         = field_length;
    field_width_          = field_width;
    goal_width_           = goal_width;
    defense_width_        = defense_width;
    defense_length_       = defense_length;
    boundary_width_       = boundary_width;
    center_circle_radius_ = center_circle_radius;
}

bool Field::valid() const
{
    return valid_;
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

Rect Field::friendlyDefenseArea() const
{
    return Rect(Point(-field_length_ * 0.5, defense_width_ / 2.0),
                Point(-field_length_ * 0.5 + defense_length_, -defense_width_ / 2.0));
}

Rect Field::enemyDefenseArea() const
{
    return Rect(Point(field_length_ * 0.5, defense_width_ / 2.0),
                Point(field_length_ * 0.5 - defense_length_, -defense_width_ / 2.0));
}

double Field::centreCircleRadius() const
{
    return center_circle_radius_;
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

std::pair<Point, Point> Field::friendlyGoalposts() const
{
    return std::make_pair(friendlyGoalpostNeg(), friendlyGoalpostPos());
}

std::pair<Point, Point> Field::enemyGoalposts() const
{
    return std::make_pair(enemyGoalpostNeg(), enemyGoalpostPos());
}

double Field::boundaryWidth() const
{
    return boundary_width_;
}

bool Field::operator==(const Field &other) const
{
    if (this->valid_ != other.valid_ || this->field_width_ != other.field_width_ ||
        this->field_length_ != other.field_length_ ||
        this->goal_width_ != other.goal_width_ ||
        this->defense_width_ != other.defense_width_ ||
        this->defense_length_ != other.defense_length_ ||
        this->boundary_width_ != other.boundary_width_ ||
        this->center_circle_radius_ != other.center_circle_radius_)
    {
        return false;
    }

    return true;
}

bool Field::operator!=(const Field &other) const
{
    return !(*this == other);
}
