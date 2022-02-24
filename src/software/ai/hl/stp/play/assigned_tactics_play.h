#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * Play that runs assigned tactics
 */
class AssignedTacticsPlay : public Play
{
   public:
    AssignedTacticsPlay(std::shared_ptr<const AiConfig> config);

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
    void updateTactics(const PlayUpdate &play_update) override;

    /**
     * Update assigned tactics for this play
     *
     * @param assigned_tactics The assigned tactics to run
     */
    void updateControlParams(std::map<RobotId, std::shared_ptr<Tactic>> assigned_tactics);

    std::vector<std::unique_ptr<Intent>> get(
        RobotToTacticAssignmentFunction,
        MotionConstraintBuildFunction motion_constraint_builder,
        const World &new_world) override;

   private:
    std::map<RobotId, std::shared_ptr<Tactic>> assigned_tactics;
};
