#include <vector>

#include "software/geom/angle_segment.h"

/**
 * Represents an AngleMap that is confined to a top and bottom angle, with angles
 * described as going from pi -> 0 -> -pi
 */
class AngleMap
{
   public:
    /**
     * Constructs an AngleMap with a specified top angle, bottom angle, and max number of
     * possible occupied AngleSegments within this map
     *
     * @param top_angle the top angle (most north) of the AngleSegment this map occupies
     * @param bottom_angle the bottom angle (most south) of the AngleSegment this map
     * occupies
     * @param max_num_obstacles the max number of possible occupied AngleSegments within
     * this map
     */
    AngleMap(Angle top_angle, Angle bottom_angle, size_t max_num_obstacles);

    /**
     * Constructs an AngleMap with a specified AngleSegment and max number of possible
     * occupied AngleSegments within this map
     *
     * @param angle_seg the AngleSegment this map occupies
     * @param max_num_obstacles the max number of possible occupied AngleSegments within
     * this map
     */
    AngleMap(AngleSegment angle_seg, size_t max_num_obstacles);

    /**
     * Gets the AngleSegment that this AngleMap occupies
     *
     * @return the AngleSegment that this AngleMap occupies
     */
    const AngleSegment &getAngleSegment() const;

    /**
     * Adds an AngleSegment to this map and marks it as occupied
     *
     * @param angleSegment the AngleSegment to mark as occupied
     */
    void addNonViableAngleSegment(AngleSegment &angle_seg);

    /**
     * Gets the biggest AngleSegment within the map that isn't occupied
     *
     * @return the biggest AngleSegment within the map that isn't occupied
     */
    AngleSegment getBiggestViableAngleSegment();

   protected:
    AngleSegment angle_seg;
    std::vector<AngleSegment> taken_angle_segments;
};
