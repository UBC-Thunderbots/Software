#pragma once
#include "software/geom/rectangle.h"
#include "software/world/field.h"

/**
 * A FieldPitchDivision is abstraction around how we split up the field.
 *
 * Pitch divisions with zones outside of the field may lead to undefined
 * behaviour. Overlapping zones are supported, but not recommended.
 */
template <class ZoneEnum>
class FieldPitchDivision
{
    static_assert(std::is_enum<ZoneEnum>::value,
                  "FieldPitchDivision: ZoneEnum must be an enum");

   public:
    FieldPitchDivision()          = default;
    virtual ~FieldPitchDivision() = default;

    /**
     * Given a zone id, return the rectangular region on the field the
     * zone id corresponds to. It is up to the implementation to
     * define the mapping between the zone_id and the corresponding region.
     *
     * NOTE: Since most soccer pitch divisions are indexed starting at 1,
     * we define zone 0 to be the entire field.
     *
     * @param zone_id The zone id
     * @return The rectangle on the field corresponding to the zone
     */
    virtual const Rectangle& getZone(ZoneEnum zone_id) const = 0;

    /**
     * Given a point, returns the Zone Id the point is located in.
     *
     * @param position The position
     * @return the Zone Id of the zone the position is in
     */
    virtual ZoneEnum getZoneId(const Point& position) const = 0;

    /**
     * Returns all the zone enums in this field pitch division
     *
     * @return A vector of all the zone ids
     */
    virtual const std::vector<ZoneEnum>& getAllZoneIds() const = 0;
};
