#include "software/world/field.h"

#include "shared/constants.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"

Field Field::createSSLDivisionBField()
{
    // Using the dimensions of a standard Division B SSL field
    // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_field_setup
    Field field = Field(9.0, 6.0, 1.0, 2.0, 0.18, 1.0, 0.3, 0.5);
    return field;
}

Field Field::createSSLDivisionAField()
{
    // Using the dimensions of a standard Division A SSL field
    // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_field_setup
    Field field = Field(12.0, 9.0, 1.8, 3.6, 0.18, 1.8, 0.3, 0.5);
    return field;
}

Field Field::createField(TbotsProto::FieldType field_type)
{
    if (field_type == TbotsProto::FieldType::DIV_A)
    {
        return createSSLDivisionAField();
    }
    else
    {
        return createSSLDivisionBField();
    }
}

Field::Field(double field_x_length, double field_y_length, double defense_x_length,
             double defense_y_length, double goal_x_length, double goal_y_length,
             double boundary_buffer_size, double center_circle_radius)
    : field_x_length_(field_x_length),
      field_y_length_(field_y_length),
      defense_x_length_(defense_x_length),
      defense_y_length_(defense_y_length),
      goal_x_length_(goal_x_length),
      goal_y_length_(goal_y_length),
      boundary_buffer_size_(boundary_buffer_size),
      center_circle_radius_(center_circle_radius),
      goal_centre_to_penalty_mark_(field_x_length_ * 2 / 3),
      enemy_defense_area(
          Point(field_x_length_ * 0.5, defense_y_length_ / 2.0),
          Point(field_x_length_ * 0.5 - defense_x_length_, -defense_y_length_ / 2.0)),
      friendly_defense_area(Rectangle(
          Point(-field_x_length_ * 0.5, defense_y_length_ / 2.0),
          Point(-field_x_length_ * 0.5 + defense_x_length_, -defense_y_length_ / 2.0))),
      field_lines(Rectangle(friendlyCornerNeg(), enemyCornerPos())),
      enemy_goal(Rectangle(
          Point(enemyGoalCenter().x(), enemyGoalpostPos().y()),
          Point(enemyGoalCenter().x() + goalXLength(), enemyGoalpostNeg().y()))),
      friendly_goal(Rectangle(
          Point(friendlyGoalCenter().x() - goalXLength(), friendlyGoalpostPos().y()),
          Point(friendlyGoalCenter().x(), friendlyGoalpostNeg().y())))
{
    if (field_x_length_ <= 0 || field_y_length <= 0 || defense_x_length_ <= 0 ||
        defense_y_length_ <= 0 || goal_x_length_ <= 0 || goal_y_length_ <= 0 ||
        boundary_buffer_size_ < 0 || center_circle_radius_ <= 0)
    {
        throw std::invalid_argument(
            "At least one field dimension is non-positive - Field is invalid");
    }
}

Field::Field(const TbotsProto::Field &field_proto)
    : Field(field_proto.field_x_length(), field_proto.field_y_length(),
            field_proto.defense_x_length(), field_proto.defense_y_length(),
            field_proto.goal_x_length(), field_proto.goal_y_length(),
            field_proto.boundary_buffer_size(), field_proto.center_circle_radius())
{
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

const Rectangle &Field::friendlyDefenseArea() const
{
    return friendly_defense_area;
}

const Rectangle &Field::enemyDefenseArea() const
{
    return enemy_defense_area;
}

Rectangle Field::friendlyHalf() const
{
    return Rectangle(friendlyCornerNeg(), Point(0, friendlyCornerPos().y()));
}

Rectangle Field::friendlyPositiveYQuadrant() const
{
    return Rectangle(friendlyGoalCenter(), Point(0, friendlyCornerPos().y()));
}

Rectangle Field::friendlyNegativeYQuadrant() const
{
    return Rectangle(friendlyGoalCenter(), Point(0, friendlyCornerNeg().y()));
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

const Rectangle &Field::fieldLines() const
{
    return field_lines;
}

Rectangle Field::fieldBoundary() const
{
    Point neg_x_neg_y_corner(-totalXLength() / 2, -totalYLength() / 2);
    Point pos_x_pos_y_corner(totalXLength() / 2, totalYLength() / 2);
    return Rectangle(neg_x_neg_y_corner, pos_x_pos_y_corner);
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

Segment Field::halfwayLine() const
{
    return Segment({0, friendlyCornerPos().y()}, {0, friendlyCornerNeg().y()});
}

Point Field::friendlyGoalCenter() const
{
    return Point(-xLength() / 2.0, 0.0);
}

Point Field::enemyGoalCenter() const
{
    return Point(xLength() / 2.0, 0.0);
}

const Rectangle &Field::friendlyGoal() const
{
    return friendly_goal;
}

const Rectangle &Field::enemyGoal() const
{
    return enemy_goal;
}

Point Field::friendlyPenaltyMark() const
{
    return Point(enemyGoalCenter().x() - goal_centre_to_penalty_mark_,
                 enemyGoalCenter().y());
}

Point Field::enemyPenaltyMark() const
{
    return Point(friendlyGoalCenter().x() + goal_centre_to_penalty_mark_,
                 friendlyGoalCenter().y());
}

Point Field::friendlyCornerPos() const
{
    return Point(friendlyGoalCenter().x(), yLength() / 2.0);
}

Point Field::friendlyCornerNeg() const
{
    return Point(friendlyGoalCenter().x(), -yLength() / 2.0);
}

Point Field::enemyCornerPos() const
{
    return Point(enemyGoalCenter().x(), yLength() / 2);
}

Point Field::enemyCornerNeg() const
{
    return Point(enemyGoalCenter().x(), -yLength() / 2);
}

Point Field::friendlyGoalpostPos() const
{
    return Point(friendlyGoalCenter().x(), goalYLength() / 2.0);
}

Point Field::friendlyGoalpostNeg() const
{
    return Point(friendlyGoalCenter().x(), -goalYLength() / 2.0);
}

Point Field::enemyGoalpostPos() const
{
    return Point(enemyGoalCenter().x(), goalYLength() / 2.0);
}

Point Field::enemyGoalpostNeg() const
{
    return Point(enemyGoalCenter().x(), -goalYLength() / 2.0);
}

double Field::boundaryMargin() const
{
    return boundary_buffer_size_;
}

bool Field::pointInFriendlyDefenseArea(const Point &p) const
{
    return contains(friendlyDefenseArea(), p);
}

bool Field::pointInEnemyDefenseArea(const Point &p) const
{
    return contains(enemyDefenseArea(), p);
}

bool Field::pointInFriendlyHalf(const Point &p) const
{
    return p.x() < centerPoint().x();
}

bool Field::pointInEnemyHalf(const Point &p) const
{
    return p.x() >= centerPoint().x();
}

bool Field::pointInFriendlyCorner(const Point &p, double radius) const
{
    return ((distance(p, friendlyCornerPos()) < radius) ||
            (distance(p, friendlyCornerNeg()) < radius)) &&
           contains(fieldLines(), p);
}

bool Field::pointInEnemyCorner(const Point &p, double radius) const
{
    return ((distance(p, enemyCornerPos()) < radius) ||
            (distance(p, enemyCornerNeg()) < radius)) &&
           contains(fieldLines(), p);
}

bool Field::operator==(const Field &other) const
{
    return this->field_y_length_ == other.field_y_length_ &&
           this->field_x_length_ == other.field_x_length_ &&
           this->goal_x_length_ == other.goal_x_length_ &&
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
