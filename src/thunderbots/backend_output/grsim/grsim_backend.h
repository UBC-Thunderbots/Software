#ifndef PROJECT_GRSIM_BACKEND_H
#define PROJECT_GRSIM_BACKEND_H
#include "backend_output/backend.h"

class GrSimBackend : public Backend
{
   public:
    explicit GrSimBackend();
    void sendPrimitives(const std::vector<Primitive> &primitives) override;
};


#endif  // PROJECT_GRSIM_BACKEND_H
