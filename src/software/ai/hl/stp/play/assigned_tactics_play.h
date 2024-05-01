#pragma once

#include "proto/parameters.pb.h"
#include "software/ai/hl/stp/play/play.h"
#include "software/ai/strategy.h"

/**
 * Play that runs assigned tactics
 */
class AssignedTacticsPlay : public Play
{
   public:
    AssignedTacticsPlay(std::shared_ptr<Strategy> strategy);

    void getNextTactics(TacticCoroutine::push_type &yield,
                        const WorldPtr &world_ptr) override;
    void updateTactics(const PlayUpdate &play_update) override;

    /**
     * Update assigned tactics for this play
     *
     * @param assigned_tactics The assigned tactics to run
     * @param motion constraints to override on a per robot id basis (if not set, then
     * build motion constraints from tactic and gamestate)
     */
    void updateControlParams(
        std::map<RobotId, std::shared_ptr<Tactic>> assigned_tactics,
        std::map<RobotId, std::set<TbotsProto::MotionConstraint>> motion_constraints =
            std::map<RobotId, std::set<TbotsProto::MotionConstraint>>());

    std::unique_ptr<TbotsProto::PrimitiveSet> get(
        const WorldPtr &world_ptr, const InterPlayCommunication &,
        const SetInterPlayCommunicationCallback &) override;

   private:
    std::map<RobotId, std::shared_ptr<Tactic>> assigned_tactics;
    std::map<RobotId, std::set<TbotsProto::MotionConstraint>> override_motion_constraints;
    RobotNavigationObstacleFactory obstacle_factory;
};
