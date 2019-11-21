/**
 * Interface of the ReceiverTactic
 */
#pragma once

#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/passing/pass.h"
#include "software/geom/ray.h"
#include "software/geom/shot.h"

/**
 * This tactic is for a robot receiving a pass. It should be used in conjunction with
 * the `PasserTactic` in order to complete the pass.
 *
 * Note that this tactic does not take into account the time the pass should occur at,
 * it simply tries to move to the best position to receive the pass as possible
 */
class ReceiverTactic : public Tactic
{
   public:
    /**
     * Creates a new ReceiverTactic
     *
     * @param field The field the pass is running on
     * @param friendly_team The friendly team
     * @param enemy_team The enemy team
     * @param pass The pass this tactic should try to receive
     * @param ball The ball being passed
     * @param loop_forever Whether or not this Tactic should never complete. If true,
     *                     the tactic will be restarted every time it completes
     */
    explicit ReceiverTactic(const Field& field, const Team& friendly_team,
                            const Team& enemy_team, const Passing::Pass pass,
                            const Ball& ball, bool loop_forever);

    std::string getName() const override;

    /**
     * Updates the world parameters for this ReceiverTactic.
     *
     * @param updated_friendly_team The current state of the friendly team
     * @param updated_enemy_team The current state of the enemy team
     * @param updated_ball The ball being passed
     */
    void updateWorldParams(const Team& updated_friendly_team,
                           const Team& updated_enemy_team, const Ball& updated_ball);

    /**
     * Updates the control parameters for this ReceiverTactic.
     *
     * @param updated_pass The pass this tactic should try to receive
     */
    void updateControlParams(const Passing::Pass& updated_pass);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

    /**
     * Calculate the angle the robot should be at in order to perform the given shot
     *
     * @param shot A Ray representing the shot we want to take
     * @param ball The ball we want to shoot
     *
     * @return The angle to position the robot at so that it can redirect the ball to
     *         the shot vector at the position where the shot vector and ball velocity
     *         vectors intersect
     */
    static Angle getOneTimeShotDirection(const Ray& shot, const Ball& ball);

    /**
     * Get the position the receiver robot should be in, and the orientation it should
     * have in order to perform a one-time shot when receiving a pass.
     *
     * @param robot The receiver robot
     * @param ball The ball
     * @param best_shot_target The point the receiver robot should shoot at
     * @return A Shot that the given robot can perform with the given ball in order to
     * shoot at the given point
     */
    static Shot getOneTimeShotPositionAndOrientation(const Robot& robot, const Ball& ball,
                                                     const Point& best_shot_target);

    void accept(TacticVisitor& visitor) const override;

   private:
    // The minimum proportion of open net we're shooting on vs the entire size of the net
    // that we require before attempting a shot
    static constexpr double MIN_SHOT_NET_PERCENT_OPEN = 0.3;

    // The maximum deflection angle that we will attempt a one-touch kick towards the
    // enemy goal with
    static constexpr Angle MAX_DEFLECTION_FOR_ONE_TOUCH_SHOT = Angle::fromDegrees(90);

    void calculateNextIntent(IntentCoroutine::push_type& yield) override;

    /**
     * Finds a feasible shot for the robot, if any.
     *
     * A feasible shot is one where the robot does not have to rotate to much to
     * take the shot, and there is a sufficient percentage of the net open for the shot.
     *
     * @return A feasible shot or std::nullopt if there is no feasible shot
     */
    std::optional<Shot> findFeasibleShot();

    // The field the pass is occuring on
    Field field;

    // The pass this tactic is executing
    Passing::Pass pass;

    // The ball being passed
    Ball ball;

    // The friendly team
    Team friendly_team;

    // The enemy team
    Team enemy_team;
};
