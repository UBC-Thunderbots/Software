#pragma once
#include "software/ai/passing/field_pitch_division.h"
#include "software/geom/rectangle.h"
#include "software/util/make_enum/make_enum.h"

class EighteenZonePitchDivision : public FieldPitchDivision
{
   public:
    // clang-format off
   /**
    *   18 Zone Pitch Division
    *      ┌────────────┬────────────┬─────────────┬─────────────┬────────────┬───────────┐
    *      │ 1          │ 4          │ 7           │ 10          │ 13         │ 16        │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      ├────────────┼────────────┼─────────────┼─────────────┼────────────┼───────────┤
    *   ┌──┤ 2          │ 5          │ 8           │ 11          │ 14         │ 17        ├──┐
    *   │  │            │            │             │             │            │           │  │
    *   │  │            │            │             │             │            │           │  │
    *   │  │            │            │             │             │            │           │  │
    *   │  │            │            │             │             │            │           │  │
    *   │  │            │            │             │             │            │           │  │
    *   │  │            │            │             │             │            │           │  │
    *   └──┤            │            │             │             │            │           ├──┘
    *      ├────────────┼────────────┼─────────────┼─────────────┼────────────┼───────────┤
    *      │ 3          │ 6          │ 9           │ 12          │ 15         │ 18        │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      │            │            │             │             │            │           │
    *      └────────────┴────────────┴─────────────┴─────────────┴────────────┴───────────┘
    *
    * The field is divided into 18 equally sized rectangles.  The enemy defense area falls
    * into in ZONE_17 and the friendly defense area is in ZONE_2.
    *
    * @param field The field to divide up into 18 zones (see unicode art above)
    */
    // clang-format on
    EighteenZonePitchDivision(const Field& field);

    const Rectangle& getZone(unsigned zone_id) override;
    inline size_t getTotalNumberOfZones(void) override;

   private:
    std::vector<Rectangle> pitch_division;
};
