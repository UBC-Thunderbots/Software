/**
 * This file contains the unit tests for the Intent class (NOTE: `Intent` is virtual, so
 * we use `MoveIntent` instead, but we're only testing functionality of the `Intent`
 * class)
 */

#include "software/ai/intent/intent.h"

#include <gtest/gtest.h>

#include "software/ai/intent/move_intent.h"


TEST(IntentTest, test_get_priority)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle(), 0.0, 2);

    EXPECT_EQ(2, move_intent.getPriority());
}

TEST(IntentTest, test_set_priority)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle(), 0.0, 0);
    move_intent.setPriority(7);

    EXPECT_EQ(7, move_intent.getPriority());
}

TEST(IntentTest, test_set_and_get_avoid_areas)
{
    MoveIntent move_intent = MoveIntent(0, Point(), Angle(), 0.0, 0);

    std::vector<AvoidArea> areas_to_avoid = {AvoidArea::FRIENDLY_DEFENSE_AREA,
                                             AvoidArea::CENTER_CIRCLE};
    move_intent.setAreasToAvoid(areas_to_avoid);

    EXPECT_EQ(areas_to_avoid, move_intent.getAreasToAvoid());
}
