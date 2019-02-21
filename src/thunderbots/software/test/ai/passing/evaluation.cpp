/**
 * This file contains unit tests passing evaluation functions
 */

#include "ai/passing/evaluation.h"
#include "test/test_util/test_util.h"

#include <gtest/gtest.h>

using namespace AI::Passing;

TEST(PassingEvaluationTest, getStaticPositionQuality_on_field_quality){
    Field f = ::Test::TestUtil::createSSLDivBField();

    // Check that the static quality is basically 0 at the edge of the field
    EXPECT_LE(getStaticPositionQuality(f, Point(-4.5, 0)), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(4.5, 0)), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(0, -3.0)), 0.13);
    EXPECT_LE(getStaticPositionQuality(f, Point(0, 3.0)), 0.13);

    // Check that we have a large static quality near the enemy goal
    EXPECT_GE(getStaticPositionQuality(f, Point(4.0, 0)), 0.80);

    // Check that we have a static quality of almost 0 near our goal
    EXPECT_LE(getStaticPositionQuality(f, Point(-4.0, 0)), 0.14);
}

TEST(PassingEvaluationTest, rectangleSigmoid){
    Rectangle r1(Point(-1,-2), Point(1, 2));

    // Check that value in the rectangle center is basically 1
    EXPECT_GE(rectangleSigmoid(r1, {0,0}, 0.1), 0.9);

    // Check that values off in x are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {-1.2,0}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.2,0}, 0.1), 0.1);

    // Check that values off in y are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {0,-2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {0,2.1}, 0.1), 0.1);

    // Check that values off in x and y are basically 0
    EXPECT_LE(rectangleSigmoid(r1, {-1.1,-2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.1,-2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {-1.1,2.1}, 0.1), 0.1);
    EXPECT_LE(rectangleSigmoid(r1, {1.1,2.1}, 0.1), 0.1);
}

TEST(PassingEvaluationTest, sigmoid){
    // Test the value after sig_width is >=0.982
    EXPECT_GE(sigmoid(10.0001, 0, 10),0.982);

    // Test the value after -sig_width is <=0.078
    EXPECT_LE(sigmoid(-10.0001, 0, 10),0.078);

    // Test that the value at 0 is 0.5 if no offset
    EXPECT_DOUBLE_EQ(0.5, sigmoid(0, 0, 1));

    // Test that the value at the offset is 0.5
    EXPECT_DOUBLE_EQ(0.5, sigmoid(2, 2, 1));
    EXPECT_DOUBLE_EQ(0.5, sigmoid(-2, -2, 1));

    // Test that negating sig_width inverts the sigmoid
    EXPECT_GE(sigmoid(-10.1, 0, -10),0.982);
    EXPECT_LE(sigmoid(10.1, 0, -10),0.078);
}


int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
