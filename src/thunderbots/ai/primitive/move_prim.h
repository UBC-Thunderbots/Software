#ifndef AI_PRIMITIVES_MOVE_H_
#define AI_PRIMITIVES_MOVE_H_

#include "ai/primitive/primitive.h"
#include "geom/angle.h"
#include "geom/point.h"
#include "thunderbots_msgs/MovePrimitive.h"

class MovePrim : public Primitive
{
   public:
    explicit MovePrim();
    explicit MovePrim(unsigned int robot_id, Point destination, Angle orientation);
    explicit MovePrim(const thunderbots_msgs::MovePrimitive &move_prim_msg);

    unsigned int robotId() const;
    Point destination() const;
    Angle orientation() const;

    thunderbots_msgs::MovePrimitive createMsg() const;

   private:
    PrimtiveID id_;
    unsigned int robot_id_;
    Point destination_;
    Angle orientation_;
};

#endif  // AI_PRIMITIVES_MOVE_H_
