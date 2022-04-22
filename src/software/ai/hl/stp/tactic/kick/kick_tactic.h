#pragma once

#include <queue>

#include "software/ai/hl/stp/tactic/kick/kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The KickTactic will move the assigned robot to the given kick origin and then
 * kick the ball to the kick target.
 */

class KickTactic : public Tactic
{
   public:
    /**
     * Creates a new KickTactic
     */
    explicit KickTactic();

    /**
     * Updates the params for this tactic that cannot be derived from the world
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param kick_speed_meters_per_second The distance between the starting location
     * of the kick and the location of the first bounce
     */
    void updateControlParams(const Point& kick_origin, const Angle& kick_direction,
                             double kick_speed_meters_per_second);

    /**
     * Updates the control parameters for this KickTactic.
     *
     * @param kick_origin The location where the kick will be taken
     * @param kick_direction The direction the Robot will kick in
     * @param kick_speed_meters_per_second The speed of how fast the Robot
     * will kick the ball in meters per second
     */
    void updateControlParams(const Point& kick_origin, const Point& kick_target,
                             double kick_speed_meters_per_second);

    void accept(TacticVisitor& visitor) const override;

    DEFINE_TACTIC_DONE_AND_GET_FSM_STATE

   private:
    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    std::map<RobotId, std::unique_ptr<FSM<KickFSM>>> fsm_map;

    // Tactic parameters
    KickFSM::ControlParams control_params;
};
