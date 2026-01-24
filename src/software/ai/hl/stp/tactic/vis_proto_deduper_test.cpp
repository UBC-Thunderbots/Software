#include "software/ai/hl/stp/tactic/vis_proto_deduper.h"
#include "software/ai/navigator/obstacle/obstacle.hpp"

#include <gtest/gtest.h>

TEST(VisProtoDeduperTest, hash_test)
{
    VisProtoDeduper deduper(10);

    auto polygon1 = Polygon({Point(4.20, 4.20), Point(1.0, 1.1), Point(-1.0, -153.52)});
    auto polygon2 = Polygon({Point(4.20, 4.20), Point(1.0, 1.1), Point(-1.0, -153.52)});

    auto polygon1_msg = createObstacleProto(polygon1);
    auto polygon2_msg = createObstacleProto(polygon2);
    EXPECT_EQ(deduper.hash(polygon1_msg), deduper.hash(polygon2_msg));
}
