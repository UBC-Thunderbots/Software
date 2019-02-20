/**
 * This file contains unit tests for the GradientDescent class
 */

#include "ai/passing/pass_generator.h"
#include "test/test_util/test_util.h"

#include <gtest/gtest.h>
#include <string.h>

using namespace AI::Passing;

TEST(GradientDescentTest, todo_test_name_here){
    PassGenerator pass_generator(0.0);

    World w = ::Test::TestUtil::createBlankTestingWorld();
    w.updateFieldGeometry(::Test::TestUtil::createSSLDivBField());

    pass_generator.setWorld(w);
    pass_generator.setPasserPoint(Point(-2, 0));

    std::this_thread::sleep_for(std::chrono::seconds(1));

    //int num_iters = 10;
    //auto pass = grad_descent.getBestPassSoFar();
    //for (auto v : pass->getParams())
    //    std::cout << v << std::endl;
    //std::cout << "====" << std::endl;
    //for (int i = 0; i < num_iters; i++){
    //    grad_descent.runGradientDescent(1);
    //    auto pass = grad_descent.getBestPassSoFar();
    //    for (auto v : pass->getParams())
    //        std::cout << v << std::endl;
    //    std::cout << "====" << std::endl;
    //}

    auto pass_op = pass_generator.getBestPassSoFar();
    if (!pass_op){
        std::cout << "No best path found" << std::endl;
    } else {
        std::cout << "Best path found!!" << std::endl;
    }

    volatile  int a =10;
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
