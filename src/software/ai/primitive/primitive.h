#pragma once

#include <string>
#include <vector>

// We forward-declare the PrimitiveVisitor interface (pure virtual class) because we need
// to know about the existence of this class in order to accept visitors with the
// accept() function. We cannot use an #include statement because this creates a cyclic
// dependency
class PrimitiveVisitor;

/**
 * Defines a Robot Primitive, which is the most basic action / unit of work a robot can
 * do. For example, moving straight to a point, pivoting around a point,
 * or shooting the ball at a target.
 *
 * This is an Abstract, pure-virtual class. It is meant to define the interface that all
 * Primitives must follow.
 * Other classes should inherit from this class and implement the methods to create a
 * useable Primitive class.
 */
class Primitive
{
   public:
    /**
     * Returns the name of the Primitive
     *
     * @return The name of the Primitive as a string
     */
    virtual std::string getPrimitiveName() const = 0;

    /**
     * Returns the ID of the robot that this Primitive corresponds
     * to / is controlling
     *
     * @return The ID of the robot this Primitive is controlling
     */
    virtual unsigned int getRobotId() const = 0;

    /**
     * Accepts a Primitive Visitor and calls the visit function
     *
     * @param visitor A Primitive Visitor
     */
    virtual void accept(PrimitiveVisitor& visitor) const = 0;

    virtual ~Primitive() = default;
};

/**
 * We use this typedef to pass lists of Primitives around
 *
 * Since `std::unique_ptr<Primitive>` does not have a copy constructor, we
 * need to pass a `shared_ptr` to the vector. To prevent the modification of
 * the vector we declare it `const`
 */
using ConstPrimitiveVectorPtr =
    std::shared_ptr<const std::vector<std::unique_ptr<Primitive>>>;
