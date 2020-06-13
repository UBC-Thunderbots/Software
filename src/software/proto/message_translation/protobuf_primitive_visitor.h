#pragma once

#include <array>
#include <optional>

#include "shared/proto/radio_primitive.pb.h"
#include "software/primitive/primitive.h"
#include "software/primitive/primitive_visitor.h"

/**
 * This class implements a Visitor that converts the Primitive classes into
 * RadioPrimitiveMsgs
 */
class ProtobufPrimitiveVisitor : public PrimitiveVisitor
{
   public:
    /**
     * Creates a new ProtobufPrimitiveVisitor
     */
    ProtobufPrimitiveVisitor() = default;

    /**
     * Visit the corresponding primitive and creates the necessary
     * protobuf for that primitive
     *
     * @param *_primitive The primitive to convert to protobuf
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
     * Converts a primitive to a RadioPrimitiveMsg
     *
     * @param The primitive to visit and convert
     * @return A unique_ptr to the respective RadioPrimitiveMsg
     */
    std::unique_ptr<RadioPrimitiveMsg> getRadioPrimitiveMsg(const Primitive &primitive);

   private:
    std::unique_ptr<RadioPrimitiveMsg> prim_msg_ptr;
};
