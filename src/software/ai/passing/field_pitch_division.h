#pragma once
#include "software/geom/rectangle.h"
#include "software/world/field.h"

/**
 * A FieldPitchDivision is abstraction around how we split up the field.
 */
class FieldPitchDivision
{
   public:
    FieldPitchDivision()          = default;
    virtual ~FieldPitchDivision() = default;

    /**
     * Given a zone id, return the rectangular region on the field the
     * zone id corresponds to. It is up to the implementation to
     * define the mapping between the zone_id and the corresponding region.
     *
     * @param zone_id The zone id
     * @return The rectangle on the field corresponding to the zone
     */
    virtual const Rectangle& getZone(unsigned zone_id) = 0;

    /**
     * Returns the total number of zones in this pitch division
     *
     * @return The total number of zones
     */
    virtual size_t getTotalNumberOfZones(void) = 0;
};
