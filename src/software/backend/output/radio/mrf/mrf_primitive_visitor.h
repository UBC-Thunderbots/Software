#pragma once

#include <array>
#include <optional>

#include "shared/firmware_primitive_type.h"
#include "software/primitive/primitive_visitor.h"

/**
 * This struct stores the components of a translated primitive to be sent over radio.
 */
typedef struct RadioPrimitive_t
{
    // A numeric ID representing the primitive for firmware
    FirmwarePrimitiveType prim_type;

    // The parameter array to be encoded into the radio packet
    std::array<double, 4> param_array;

    // Extra bits used for flags and/or additional information
    uint8_t extra_bits;

    // Indicates whether the robot should move slowly (<1.5 m/s)
    bool slow;
} RadioPrimitive;

inline bool operator==(const RadioPrimitive &lhs, const RadioPrimitive &rhs)
{
    return lhs.prim_type == rhs.prim_type && lhs.param_array == rhs.param_array &&
           lhs.extra_bits == rhs.extra_bits;
}

/**
 * This class implements a Visitor that serializes the Primitive classes into packets
 * that can be send over the radio to the physical robots
 */
class MRFPrimitiveVisitor : public PrimitiveVisitor
{
   public:
    /**
     * Creates a new RadioSerializerPrimitiveVisitor
     */
    MRFPrimitiveVisitor() = default;

    /**
     * Serializes the given Primitive into a radio packet
     *
     * @param The Primitive to serialize
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
     * Returns the most recent serialized packet created by this
     * MRFPrimitiveVisitor (std::nullopt if no packet has been created).
     *
     * This is the radio packet created by one of the 'visit' functions.
     *
     * @return The most recent serialized packet created by this
     * MRFPrimitiveVisitor
     */
    RadioPrimitive getSerializedRadioPacket();

   private:
    std::optional<RadioPrimitive> radio_prim;
};
