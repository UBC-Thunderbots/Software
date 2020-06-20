#pragma once

#include <array>
#include <optional>

#include "software/primitive/primitive_visitor.h"
#include "shared/proto/radio_primitive.pb.h"

// TODO: delete this
///**
// * This struct stores the components of a translated primitive to be sent over radio.
// */
//typedef struct RadioPrimitive_t
//{
//    // A numeric ID representing the primitive for firmware
//    FirmwarePrimitiveType prim_type;
//
//    // The parameter array to be encoded into the radio packet
//    std::array<double, 4> param_array;
//
//    // Extra bits used for flags and/or additional information
//    uint8_t extra_bits;
//
//    // Indicates whether the robot should move slowly (<1.5 m/s)
//    bool slow;
//} RadioPrimitive;
//
// TODO: delete this
//inline bool operator==(const RadioPrimitive &lhs, const RadioPrimitive &rhs)
//{
//    return lhs.prim_type == rhs.prim_type && lhs.param_array == rhs.param_array &&
//           lhs.extra_bits == rhs.extra_bits;
//}

/**
 * This class implements a Visitor that serializes the Primitive classes into packets
 * that can be send over the radio to the physical robots
 */
 // TODO: better name for this class
 // TODO: rename files to match class name
class CreateProtoPrimitiveVisitor : public PrimitiveVisitor
{
   public:
    CreateProtoPrimitiveVisitor() = default;

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
     * Get the proto representation of the most recently visited primitive
     *
     * @throws std::runtime_error If this visitor has never visited a primitive
     *
     * @return The proto representation of the most recently visited primitive
     */
    RadioPrimitiveMsg getProto();

   private:
    std::optional<RadioPrimitiveMsg> radio_prim;
};
