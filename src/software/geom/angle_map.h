#include <vector>

#include "software/geom/angle_segment.h"

/**
 * Represents an AngleMap that is confined to a top and bottom angle, with angles
 * described as going from pi -> 0 -> -pi
 *
 * NOTE:
 * If we have two angle segments:
 * a = 0 -> 45
 * b = 90 -> 180
 *
 * These two get added to the map
 *
 * Now we have another angle segment:
 * c = 45 -> 90
 *
 * If c gets added to the map it will only merge with one of a or b, not both. Merging
 * with the one that was placed in the map first
 *
 * The map will still have two angle segments even though it should only have one since
 * they all overlap. Sorting before hand solves this so they'll all be in an incremental
 * order
 */
class AngleMap
{
   public:
    /**
     * Constructs an AngleMap with a specified top angle, bottom angle, and max number of
     * possible occupied AngleSegments within this map
     *
     * @param top_angle the top angle (most positive) of the AngleSegment this map
     * occupies
     * @param bottom_angle the bottom angle (most negative) of the AngleSegment this map
     * occupies
     * @param max_num_obstacles the max number of possible occupied AngleSegments within
     * this map
     */
    AngleMap(Angle top_angle, Angle bottom_angle, size_t reserved_num_obstacles = 1);

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
    void addNonViableAngleSegment(AngleSegment &obstacle_angle_seg);

    /**
     * Gets the biggest AngleSegment within the map that isn't occupied
     *
     * @return the biggest AngleSegment within the map that isn't occupied
     */
    AngleSegment getBiggestViableAngleSegment();

   private:
    AngleSegment angle_seg;
    std::vector<AngleSegment> taken_angle_segments;
};
