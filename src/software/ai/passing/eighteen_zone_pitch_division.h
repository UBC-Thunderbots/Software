#pragma once
#include <memory>

#include "software/ai/passing/field_pitch_division.h"
#include "software/geom/rectangle.h"
#include "software/util/make_enum/make_enum.h"

// clang-format off
//
// 18 Zone Pitch Division
// 
//                FRIENDLY          ENEMY
//        ┌──────┬──────┬──────┬──────┬──────┬─────┐
//        │1     │4     │7     │10    │13    │16   │
//        │      │      │      │      │      │     │
//        │      │      │      │      │      │     │
//        ├──────┼──────┼──────┼──────┼──────┼─────┤
//      ┌─┤2     │5     │8     │11    │14    │17   ├─┐
//      │ │      │      │      │      │      │     │ │
//      │ │      │      │      │      │      │     │ │
//      └─┤      │      │      │      │      │     ├─┘
//        ├──────┼──────┼──────┼──────┼──────┼─────┤
//        │3     │6     │9     │12    │15    │18   │
//        │      │      │      │      │      │     │
//        │      │      │      │      │      │     │
//        └──────┴──────┴──────┴──────┴──────┴─────┘
//
MAKE_ENUM(EighteenZoneId,
            ZONE_1, ZONE_2, ZONE_3, ZONE_4, ZONE_5,
            ZONE_6, ZONE_7, ZONE_8, ZONE_9, ZONE_10,
            ZONE_11, ZONE_12, ZONE_13, ZONE_14, ZONE_15,
            ZONE_16, ZONE_17, ZONE_18);
// clang-format on

class EighteenZonePitchDivision : public FieldPitchDivision<EighteenZoneId>
{
   public:
    /**
     * The field is divided into 18 equally sized rectangles.
     *
     * @param field The field to divide up into 18 zones (see ascii art above)
     */
    EighteenZonePitchDivision(const Field& field);

    const Rectangle& getZone(EighteenZoneId zone_id) const override;
    const std::vector<EighteenZoneId>& getAllZoneIds() const override;
    EighteenZoneId getZoneId(const Point& position) const override;

   private:
    std::shared_ptr<Rectangle> field_lines_;
    std::vector<Rectangle> pitch_division_;
    std::vector<EighteenZoneId> zones_;
};
