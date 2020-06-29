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
    void visit(const CatchPrimitive &catch_primitive) override;
    void visit(const ChipPrimitive &chip_primitive) override;
    void visit(const DirectVelocityPrimitive &direct_velocity_primitive) override;
    void visit(const DirectWheelsPrimitive &direct_wheels_primitive) override;
    void visit(const DribblePrimitive &dribble_primitive) override;
    void visit(const KickPrimitive &kick_primitive) override;
    void visit(const MovePrimitive &move_primitive) override;
    void visit(const MoveSpinPrimitive &movespin_primitive) override;
    void visit(const PivotPrimitive &pivot_primitive) override;
    void visit(const StopPrimitive &stop_primitive) override;

    /**
     * Create a PrimitiveMsg from the given Primitive
     * @param primitive The primitive to create the PrimitiveMsg from
     * @return A PrimitiveMsg representation of the given primitive
     */
    PrimitiveMsg createPrimitiveMsg(const Primitive &primitive);

   private:
    /**
     * Get the proto representation of the most recently visited primitive
     *
     * @throws std::runtime_error If this visitor has never visited a primitive
     *
     * @return The proto representation of the most recently visited primitive
     */
    PrimitiveMsg getProto();

    std::optional<PrimitiveMsg> prim;
};
