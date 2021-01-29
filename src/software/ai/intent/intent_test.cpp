#include "software/ai/intent/intent.h"

#include <gtest/gtest.h>

#include "software/ai/intent/stop_intent.h"

/**
 * This file contains the unit tests for the Intent class (NOTE: `Intent` is virtual, so
 * we use `StopIntent` instead, but we're only testing functionality of
 * the `Intent` class)
 */

TEST(IntentTest, test_get_robot_id)
{
    StopIntent stop_intent = StopIntent(1, false);
    EXPECT_EQ(stop_intent.getRobotId(), 1);
}

TEST(IntentTest, test_robot_id_inequality)
{
    StopIntent stop_intent_1 = StopIntent(0, false);
    StopIntent stop_intent_2 = StopIntent(1, false);

    EXPECT_NE(stop_intent_1, stop_intent_2);
}
