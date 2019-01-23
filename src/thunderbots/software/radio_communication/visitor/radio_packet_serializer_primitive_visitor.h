#pragma once

#include "ai/primitive/visitor/primitive_visitor.h"

/**
 * This class implements a Visitor that serializes the Primitive classes into packets
 * that can be send over the radio to the physical robots
 */
class RadioPacketSerializerPrimitiveVisitor : public PrimitiveVisitor
{
   public:
    /**
     * Creates a new RadioSerializerPrimitiveVisitor
     */
    RadioPacketSerializerPrimitiveVisitor() = default;

    /**
     * Serializes the given CatchPrimitive into a radio packet
     *
     * @param catch_primitive The CatchPrimitive to serialize
     */
    void visit(const CatchPrimitive &catch_primtiive) override;

    /**
     * Serializes the given ChipPrimitive into a radio packet
     *
     * @param chip_primitive The ChipPrimitive to simulate
     */
    void visit(const ChipPrimitive &chip_primtiive) override;

    /**
     * Serializes the given DirectVelocityPrimitive into a radio packet
     *
     * @param direct_velocity_primitive The DirectVelocityPrimitive to simulate
     */
    void visit(const DirectVelocityPrimitive &direct_velocity_primtiive) override;

    /**
     * Serializes the given KickPrimitive into a radio packet
     *
     * @param kick_primitive The KickPrimitive to simulate
     */
    void visit(const KickPrimitive &kick_primtiive) override;

    /**
     * Serializes the given MovePrimitive into a radio packet
     *
     * @param move_primitive The MovePrimitive to simulate
     */
    void visit(const MovePrimitive &move_primitive) override;

    /**
     * Serializes the given PivotPrimitive into a radio packet
     *
     * @param pivot_primitive The PivotPrimitive to simulate
     */
    void visit(const PivotPrimitive &pivot_primtiive) override;

    /**
     * Returns the most recent serialized packet created by this
     * RadioPacketSerializerPrimitiveVisitor. This is the radio packet created by
     * one of the 'visit' functions
     *
     * Calling this function before this Visitor has been called at least once has
     * undefined behavior
     *
     * @return The most recent serialized packet created by this
     * RadioPacketSerializerPrimitiveVisitor
     */
    uint64_t getSerializedRadioPacket();

   private:
    // The serialzed data encoded to tbe sent to the robots over radio
    // TODO: This may not be the type actually required for the radio
    // See: https://github.com/UBC-Thunderbots/Software/issues/63
    uint64_t radio_packet;
};
