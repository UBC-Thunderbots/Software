#pragma once

#include "software/ai/hl/stp/skill/skill_fsm.h"
#include "software/ai/strategy.h"

class Skill
{
   public:
    explicit Skill(std::shared_ptr<Strategy> strategy) : strategy_(strategy){};

    /**
     * Calls the SetPrimitiveCallback with a primitive for the given robot.
     * The primitive is the result of a single step of execution of the Skill for the
     * robot.
     *
     * @param robot the robot that will execute the Skill
     * @param world_ptr the world pointer
     * @param set_primitive callback function to set the primitive for the robot
     */
    virtual void updatePrimitive(const Robot& robot, const WorldPtr& world_ptr,
                                 const SetPrimitiveCallback& set_primitive) = 0;

    /**
     * Resets the state of the Skill for the given robot.
     *
     * @param robot the robot
     */
    virtual void reset(const Robot& robot) = 0;

    /**
     * Returns whether the given robot has finished executing the Skill.
     *
     * @param robot the robot executing the Skill
     *
     * @return true if the robot has finished executing the Skill, false otherwise
     */
    virtual bool done(const RobotId& robot_id) const = 0;

    virtual std::string getFSMState(RobotId robot_id) const = 0;

   protected:
    std::shared_ptr<Strategy> strategy_;
};
