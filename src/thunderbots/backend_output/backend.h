#ifndef PROJECT_BACKEND_H
#define PROJECT_BACKEND_H

#include "ai/primitive/primitive.h"

class Backend {
public:
    virtual void sendPrimitives(const std::vector<Primitive> &primitives);
};



#endif //PROJECT_BACKEND_H
