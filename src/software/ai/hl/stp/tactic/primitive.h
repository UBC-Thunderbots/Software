#pragma once

#include "proto/primitive.pb.h"

/**
 * The primitive actions that a robot can perform
 */
class Primitive
{
public:
    /**
     * Destructor
     */
    virtual ~Primitive() = default;

    /**
    * Gets the primitive proto message
    *
    * @return the primitive proto message
    */
    virtual std::unique_ptr<TbotsProto::Primitive> generatePrimitiveProtoMessage() = 0;

    /**
     * Gets the estimated cost of the primitive
     *
     * @return estimated cost of the primitive
     */
    double getEstimatedPrimitiveCost() const
    {
        return estimated_cost;
    }

protected:
    double estimated_cost = 0;
};