#pragma once

#include "software/geom/angle.h"
#include "software/geom/point.h"

/**
 * A pose in 2D space which has a position and an orientation
 */
class Pose final
{
   public:
    /**
     * Creates a Pose at the origin (0, 0) with 0 degrees orientation.
     */
    explicit Pose();

    /**
     * Creates a new Point that is a copy of the given Point
     *
     * @param the Point to duplicate
     */
    Pose(const Point &position, const Angle orientation);

    /**
     * Returns the position of this Pose
     * @return the position of this Pose
     */
    const Point &position() const;

    /**
     * Returns the orientation of this Pose
     * @return the orientation of this Pose
     */
    const Angle &orientation() const;

    /**
     * Compares another Pose with this Pose for equality
     * @param rhs the other Pose to compare with
     * @return true if the two Poses are equal, false otherwise
     */
    bool operator==(const Pose &rhs) const;

    /**
     * Compares another Pose with this Pose for inequality
     * @param rhs the other Pose to compare with
     * @return true if the two Poses are not equal, false otherwise
     */
    bool operator!=(const Pose &rhs) const;

private:
    /**
     * The position of the Pose. The variable name starts with an underscore to prevent
     * name conflicts with its accessor function.
     */
    Point position_;

    /**
     * The orientation of the Pose. The variable name starts with an underscore to prevent
     * name conflicts with its accessor function.
     */
    Angle orientation_;
};

/**
 * Prints a pose to a stream
 *
 * @param os the stream to print to
 * @param p the Pose to print
 *
 * @return the stream with the pose printed
 */
std::ostream &operator<<(std::ostream &os, const Point &p);
