#pragma once

#include "software/ai/intent/intent.h"

/**
 * A DirectPrimitiveIntent is an Intent that directly represents a Primitive, which means
 * no additional work is needed to create the primitive
 */
class DirectPrimitiveIntent : public Intent
{
   public:
    /**
     * Creates a new DirectPrimitiveIntent
     *
     * @param robot_id The id of the Robot to run this Primitive
     * @param priority The priority of this Intent. A larger number indicates a higher
     * priority
     * @param the TbotsProto::Primitive directly underlying this Intent
     */
    explicit DirectPrimitiveIntent(unsigned int robot_id, unsigned int priority,
                                   TbotsProto::Primitive primitive_msg);

    void accept(IntentVisitor& visitor) const override;

    /**
     * Compares DirectPrimitiveIntents for equality. DirectPrimitiveIntents are considered
     * equal if all their member variables are equal.
     *
     * @param other the DirectPrimitiveIntents to compare with for equality
     *
     * @return true if the DirectPrimitiveIntents are equal and false otherwise
     */
    bool operator==(const DirectPrimitiveIntent& other) const;

    /**
     * Compares DirectPrimitiveIntents for inequality.
     *
     * @param other the DirectPrimitiveIntent to compare with for inequality
     *
     * @return true if the DirectPrimitiveIntents are not equal and false otherwise
     */
    bool operator!=(const DirectPrimitiveIntent& other) const;

    /**
     * Directly gets the Primitive for this Intent
     *
     * @return The Primitive that represents this Intent
     */
    TbotsProto::Primitive getPrimitive() const;

   private:
    TbotsProto::Primitive primitive_msg;
};
