#include <gtest/gtest.h>
#include "software/ai/passing/eighteen_zone_pitch_division.h"
#include "software/world/field.h"
#include "software/geom/algorithms/contains.h"


TEST(EighteenZonePitchDivision, test_pitch_division_div_a)
{
    auto field = Field::createSSLDivisionAField();
    auto pitch_division = EighteenZonePitchDivision(field);

    // from the pitch divison diagram in eighteen_zone_pitch_division.h
    // we know that zones 1 - 9 should be on the friendly side and 
    // zones 10 - 18 should be on the enemy side.
    for (unsigned zone = 1; zone <= 9; ++zone)
    {
        ASSERT_TRUE(contains(field.friendlyHalf(), pitch_division.getZone(zone).centre())) << zone;
    }

    for (unsigned zone = 10; zone <= 18; ++zone)
    {
        ASSERT_TRUE(contains(field.enemyHalf(), pitch_division.getZone(zone).centre()));
    }
}

TEST(EighteenZonePitchDivision, test_pitch_division_div_b)
{
    auto field = Field::createSSLDivisionBField();
    auto pitch_division = EighteenZonePitchDivision(field);

    // from the pitch divison diagram in eighteen_zone_pitch_division.h
    // we know that zones 1 - 9 should be on the friendly side and 
    // zones 10 - 18 should be on the enemy side.
    for (unsigned zone = 1; zone <= 9; ++zone)
    {
        ASSERT_TRUE(contains(field.friendlyHalf(), pitch_division.getZone(zone).centre())) << zone;
    }

    for (unsigned zone = 10; zone <= 18; ++zone)
    {
        ASSERT_TRUE(contains(field.enemyHalf(), pitch_division.getZone(zone).centre()));
    }
}
