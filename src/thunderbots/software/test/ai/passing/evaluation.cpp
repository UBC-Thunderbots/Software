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

    int num_iters = 100;
    for (int i = 0; i < num_iters; i++){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto pass_op = pass_generator.getBestPassSoFar();
        if (!pass_op){
            std::cout << "No best path found" << std::endl;
        } else {
            std::cout << *pass_op << std::endl;
        }
    }

    auto pass_op = pass_generator.getBestPassSoFar();
    if (!pass_op){
        std::cout << "No best path found" << std::endl;
    } else {
        std::cout << "Best path found!!" << std::endl;
    }

    volatile  int a =10;
}

TEST(GradientDescentTest, todo_test_name_here2) {
    PassGenerator pass_generator(0.0);

    World w = ::Test::TestUtil::createBlankTestingWorld();
    w.updateFieldGeometry(::Test::TestUtil::createSSLDivBField());
    pass_generator.setWorld(w);
    pass_generator.setPasserPoint(Point(-2, 0));

}

TEST(GradientDescentTest, todo_test_name_here1){
    PassGenerator pass_generator(0.8);

    World w = ::Test::TestUtil::createBlankTestingWorld();
    w.updateFieldGeometry(::Test::TestUtil::createSSLDivBField());

    pass_generator.setWorld(w);
    pass_generator.setPasserPoint(Point(-2, 0));

    std::optional<Pass> pass_op = std::nullopt;
    while(!pass_op){
        // yield()
        pass_op = pass_generator.getBestPassSoFar();
    }
    std::cout << "Found Pass: " << *pass_op << std::endl;
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
