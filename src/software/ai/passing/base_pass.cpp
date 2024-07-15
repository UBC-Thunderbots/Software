#include "software/ai/passing/base_pass.h"

BasePass::BasePass(Point passer_point, Point receiver_point)
    : passer_point(passer_point),
      receiver_point(receiver_point) {}

Point BasePass::receiverPoint() const
{
    return receiver_point;
}

Angle BasePass::receiverOrientation() const
{
    return (passerPoint() - receiverPoint()).orientation();
}

Angle BasePass::passerOrientation() const
{
    return (receiverPoint() - passerPoint()).orientation();
}

Point BasePass::passerPoint() const
{
    return passer_point;
}

double BasePass::length() const 
{
    return std::sqrt(std::pow(receiverPoint().x() - passerPoint().x(), 2) + std::pow(receiverPoint().y() - passerPoint().y(), 2));
}

std::array<double, NUM_PARAMS_TO_OPTIMIZE> BasePass::toPassArray() const
{
    return {receiver_point.x(), receiver_point.y()};
}

bool BasePass::operator==(const BasePass& other) const
{
    return this->passer_point == other.passer_point &&
           this->receiver_point == other.receiver_point;
}