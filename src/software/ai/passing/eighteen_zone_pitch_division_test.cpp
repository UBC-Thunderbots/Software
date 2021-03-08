#include "software/ai/passing/eighteen_zone_pitch_division.h"

#include <gtest/gtest.h>

#include "software/geom/algorithms/contains.h"
#include "software/world/field.h"


TEST(EighteenZonePitchDivision, test_pitch_division_div_a)
{
    auto field          = Field::createSSLDivisionAField();
    auto pitch_division = EighteenZonePitchDivision(field);

    // from the pitch divison diagram in eighteen_zone_pitch_division.h
    // we know that zones 1 - 9 should be on the friendly side and
    // zones 10 - 18 should be on the enemy side.
    for (EighteenZonePitchId zone : {
             EighteenZonePitchId::ZONE_1,
             EighteenZonePitchId::ZONE_2,
             EighteenZonePitchId::ZONE_3,
             EighteenZonePitchId::ZONE_4,
             EighteenZonePitchId::ZONE_5,
             EighteenZonePitchId::ZONE_6,
             EighteenZonePitchId::ZONE_7,
             EighteenZonePitchId::ZONE_8,
             EighteenZonePitchId::ZONE_9,
         })
    {
        ASSERT_TRUE(
            contains(field.friendlyHalf(), pitch_division.getZone(zone).centre()));
    }

    for (EighteenZonePitchId zone : {
             EighteenZonePitchId::ZONE_10,
             EighteenZonePitchId::ZONE_11,
             EighteenZonePitchId::ZONE_12,
             EighteenZonePitchId::ZONE_13,
             EighteenZonePitchId::ZONE_14,
             EighteenZonePitchId::ZONE_15,
             EighteenZonePitchId::ZONE_16,
             EighteenZonePitchId::ZONE_17,
             EighteenZonePitchId::ZONE_18,
         })
    {
        ASSERT_TRUE(contains(field.enemyHalf(), pitch_division.getZone(zone).centre()));
    }
}

TEST(EighteenZonePitchDivision, test_pitch_division_div_b)
{
    auto field          = Field::createSSLDivisionBField();
    auto pitch_division = EighteenZonePitchDivision(field);

    // from the pitch divison diagram in eighteen_zone_pitch_division.h
    // we know that zones 1 - 9 should be on the friendly side and
    // zones 10 - 18 should be on the enemy side.
    for (EighteenZonePitchId zone : {
             EighteenZonePitchId::ZONE_1,
             EighteenZonePitchId::ZONE_2,
             EighteenZonePitchId::ZONE_3,
             EighteenZonePitchId::ZONE_4,
             EighteenZonePitchId::ZONE_5,
             EighteenZonePitchId::ZONE_6,
             EighteenZonePitchId::ZONE_7,
             EighteenZonePitchId::ZONE_8,
             EighteenZonePitchId::ZONE_9,
         })
    {
        ASSERT_TRUE(
            contains(field.friendlyHalf(), pitch_division.getZone(zone).centre()));
    }

    for (EighteenZonePitchId zone : {
             EighteenZonePitchId::ZONE_10,
             EighteenZonePitchId::ZONE_11,
             EighteenZonePitchId::ZONE_12,
             EighteenZonePitchId::ZONE_13,
             EighteenZonePitchId::ZONE_14,
             EighteenZonePitchId::ZONE_15,
             EighteenZonePitchId::ZONE_16,
             EighteenZonePitchId::ZONE_17,
             EighteenZonePitchId::ZONE_18,
         })
    {
        ASSERT_TRUE(contains(field.enemyHalf(), pitch_division.getZone(zone).centre()));
    }
}
