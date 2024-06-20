#pragma once

#include "software/ai/hl/stp/skill/skill_fsm.h"
#include "software/ai/hl/stp/skill/skill_state.h"
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
     * @param robot_id the ID of the robot executing the Skill
     *
     * @return true if the robot has finished executing the Skill, false otherwise
     */
    virtual bool done(const RobotId robot_id) const = 0;

    /**
     * Returns whether the given robot has temporarily suspended execution of the Skill.
     *
     * IMPORTANT: updatePrimitive should not be called with a robot that has
     * suspended execution of the Skill. This is because a suspended Skill
     * will not yield any primitives for the robot to execute.
     *
     * @param robot_id the ID of the robot executing the Skill
     *
     * @return true if the robot has suspended execution of the Skill, false otherwise
     */
    virtual bool suspended(const RobotId robot_id) const = 0;

    /**
     * If the given robot has temporarily suspended execution of the Skill, calling this
     * method will update the Skill with the given world and try to resume execution
     * (i.e. leave the suspended state).
     *
     * If the Skill is not suspended for the given robot, then calling this method
     * effectively does nothing.
     *
     * @param robot_id the ID of the robot executing the Skill
     * @param world_ptr the world pointer
     *
     * @return true if the Skill is still suspended for the robot after
     * trying to resume execution, false otherwise
     */
    virtual bool tryResumingIfSuspended(const RobotId robot_id,
                                        const WorldPtr& world_ptr) = 0;

    /**
     * Gets the FSM state of the Skill for the given robot.
     * 
     * @param robot_id the ID of the robot
     * 
     * @return the FSM state for the given robot
     */
    virtual std::string getFSMState(const RobotId robot_id) const = 0;

    /**
     * Gets a SkillState containing details about the current state of the skill
     * for the given robot.
     * 
     * @param robot_id the ID of the robot
     * 
     * @return the SkillState for the given robot 
     */
    virtual SkillState getSkillState(const RobotId robot_id) const = 0;

   protected:
    std::shared_ptr<Strategy> strategy_;
};
