#pragma once

#include "software/ai/hl/stp/tactic/primitive.h"
#include "proto/primitive.pb.h"
#include <memory>

class StopPrimitive : public Primitive
{
public:
    StopPrimitive() = default;
    ~StopPrimitive() override = default;

    /**
    * Gets the primitive proto message
    *
    * @return the primitive proto message
    */
    std::unique_ptr<TbotsProto::Primitive> generatePrimitiveProtoMessage() override;
};
