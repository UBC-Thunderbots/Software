#pragma once

#include "ai/primitive/primitive.h"

class Backend
{
   public:
    /**
     * Sends the given primitives to the backend to control the robots
     * @param primitives the list of primitives to send
     */
    virtual void sendPrimitives(const std::vector<Primitive> &primitives);
};
