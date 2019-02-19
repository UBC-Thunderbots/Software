/**
 * This file contains unit tests for the GradientDescent class
 */

#include "ai/passing/passing_gradient_descent.h"
#include "test/test_util/test_util.h"

#include <gtest/gtest.h>
#include <string.h>

using namespace AI::Passing;

TEST(GradientDescentTest, todo_test_name_here){
    GradientDescent grad_descent;

    World w = ::Test::TestUtil::createBlankTestingWorld();
    w.updateFieldGeometry(::Test::TestUtil::createSSLDivBField());

    grad_descent.setWorld(w);
    grad_descent.setPasserPoint(Point(-2, 0));

    grad_descent.runGradientDescent(10000);

    int num_iters = 10;
    auto pass = grad_descent.getBestPass();
    for (auto v : pass->getParams())
        std::cout << v << std::endl;
    std::cout << "====" << std::endl;
    for (int i = 0; i < num_iters; i++){
        grad_descent.runGradientDescent(1);
        auto pass = grad_descent.getBestPass();
        for (auto v : pass->getParams())
            std::cout << v << std::endl;
        std::cout << "====" << std::endl;
    }

    volatile  int a =10;
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
