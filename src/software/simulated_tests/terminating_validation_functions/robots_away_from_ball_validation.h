#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks that all robots move away a given distance from the ball.
 * 
 * @param min_distance the minimum distance that the robots should move from the ball
 * @param excluded_robots a list of RobotIds to ignore in this validation function's checking
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield yields control to the next routine (coroutines)
 */
void robotsAwayFromBall(double min_distance, std::vector<RobotId> excluded_robots, 
                        std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield);