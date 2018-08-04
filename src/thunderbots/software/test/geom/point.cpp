#include "geom/point.h"
#include <gtest/gtest.h>

TEST(AngleTest, accesors)
{
    EXPECT_DOUBLE_EQ(0, Point().x());
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
