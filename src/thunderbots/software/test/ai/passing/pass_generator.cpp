/**
 * this file contains unit tests for the gradientdescent class
 */

#include "ai/passing/pass_generator.h"

#include <gtest/gtest.h>
#include <string.h>

#include "test/test_util/test_util.h"

using namespace ai::passing;

class passgeneratortest : public testing::test
{
   protected:
   protected:
    virtual void setup()
    {
        world = ::test::testutil::createblanktestingworld();
        world.updatefieldgeometry(::test::testutil::createssldivbfield());
        pass_generator = std::make_shared<passgenerator>(0.0);
        pass_generator->setworld(world);
    }

    world world;
    std::shared_ptr<passgenerator> pass_generator;
};

test_f(passgeneratortest, static_convergence_towards_target_region)
{
    // test that given enough time and a static world with no robots, we converge to a
    // pass near the enemy team goal

    std::this_thread::sleep_for(std::chrono::seconds(5));

    std::optional<pass> pass1 = pass_generator->getbestpasssofar();

    // make sure we got some pass
    assert_true(pass1);

    // check that the pass is across the half-line towards the enemy goal
    expect_ge(pass1->receiverpoint().x(), 0.1);
    // currently we just generate receiver points at (0,0), so y should be 0
    expect_eq(pass1->receiverpoint().y(), 0);

    // run a bit more
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::optional<pass> pass2 = pass_generator->getbestpasssofar();

    // check that we're moving towards the goal
    assert_true(pass2);
    expect_ge(pass2->receiverpoint().x(), pass1->receiverpoint().x());
    // currently we just generate receiver points at (0,0), so y should be 0
    expect_eq(pass2->receiverpoint().y(), 0);

    std::cout << pass1->receiverpoint() << std::endl;
    std::cout << pass2->receiverpoint() << std::endl;

    // todo (issue #323): check more things here when we're actually generating passes
}

int main(int argc, char **argv)
{
    std::cout << argv[0] << std::endl;
    testing::initgoogletest(&argc, argv);
    return run_all_tests();
}
