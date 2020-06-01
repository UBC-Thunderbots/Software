#pragma once

#include <array>
#include <optional>

#include "shared/proto/radio_primitive.pb.h"
#include "software/primitive/primitive_visitor.h"

/**
 * This class implements a Visitor that serializes the Primitive classes into packets
 * that can be send over the radio to the physical robots
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
     * Returns a unique_ptr RadioPrimitiveMsg
     *
     * @return The unique_ptr to most recently constructed msg
     * created by this ProtobufPrimitiveVisitor
     */
    std::unique_ptr<RadioPrimitiveMsg> getRadioPrimitiveMsg();

   private:
    std::unique_ptr<RadioPrimitiveMsg> prim_msg_ptr;
};
