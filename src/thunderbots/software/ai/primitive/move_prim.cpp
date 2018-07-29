#include "move_prim.h"

MovePrim::MovePrim() : id_(PRIM_MOVE_ID)
{
}

MovePrim::MovePrim(unsigned int robot_id, Point destination, Angle orientation)
{
    id_          = PRIM_MOVE_ID;
    robot_id_    = robot_id;
    destination_ = destination;
    orientation_ = orientation;
}

MovePrim::MovePrim(const thunderbots_msgs::MovePrimitive &move_prim_msg)
{
    id_          = PRIM_MOVE_ID;
    robot_id_    = move_prim_msg.robot_id;
    destination_ = Point(move_prim_msg.destination.x, move_prim_msg.destination.y);
    orientation_ = Angle::ofRadians(move_prim_msg.orientation);
}

unsigned int MovePrim::robotId() const
{
    return robot_id_;
}

Point MovePrim::destination() const
{
    return destination_;
}

Angle MovePrim::orientation() const
{
    return orientation_;
}

thunderbots_msgs::MovePrimitive MovePrim::createMsg() const
{
    thunderbots_msgs::MovePrimitive msg;
    msg.robot_id      = robot_id_;
    msg.prim_id       = id_;
    msg.destination.x = destination_.x();
    msg.destination.y = destination_.y();
    msg.orientation   = orientation_.toRadians();
    return msg;
}