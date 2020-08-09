#pragma once

#include <array>
#include <optional>

#include "shared/proto/primitive.pb.h"
#include "software/primitive/primitive.h"
#include "software/primitive/primitive_visitor.h"

/**
 * This class allows for the creation of a ProtoBuf message for a given primitive
 */
class ProtoCreatorPrimitiveVisitor : public PrimitiveVisitor
{
   public:
    ProtoCreatorPrimitiveVisitor() = default;

    /**
     * Serializes the given Primitive into a radio packet
     * * @param The Primitive to serialize */

    /**
     * Visits a given primitive
     */
    void visit(const ChipPrimitive &chip_primitive) override;
    void visit(const DirectVelocityPrimitive &direct_velocity_primitive) override;
    void visit(const DirectWheelsPrimitive &direct_wheels_primitive) override;
    void visit(const KickPrimitive &kick_primitive) override;
    void visit(const MovePrimitive &move_primitive) override;
    void visit(const SpinningMovePrimitive &spinning_move_primitive) override;
    void visit(const StopPrimitive &stop_primitive) override;

    /**
     * Create a TbotsProto::Primitive from the given Primitive
     *
     * @param primitive The primitive to create the TbotsProto::Primitive from
     *
     * @return A TbotsProto::Primitive representation of the given primitive
     */
    TbotsProto::Primitive createPrimitive(const Primitive &primitive);

   private:
    /**
     * Get the proto representation of the most recently visited primitive
     *
     * @throws std::runtime_error If this visitor has never visited a primitive
     *
     * @return The proto representation of the most recently visited primitive
     */
    TbotsProto::Primitive getProto();

    std::optional<TbotsProto::Primitive> prim;
};
