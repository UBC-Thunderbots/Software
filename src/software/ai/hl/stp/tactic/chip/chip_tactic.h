#pragma once

#include <queue>

#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"

/**
 * The ChipTactic will move the assigned robot to the given chip origin and then
 * chip the ball to the chip target.
 */

class ChipTactic : public Tactic<ChipFSM>
{
   public:
    /**
     * Creates a new ChipTactic
     *
     * @param ai_config_ptr shared pointer to ai_config
     */
    explicit ChipTactic(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr);

    /**
     * Updates the params for this tactic that cannot be derived from the world
     *
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The direction the Robot will chip in
     * @param chip_distance_meters The distance between the starting location
     * of the chip and the location of the first bounce
     */
    void updateControlParams(const Point& chip_origin, const Angle& chip_direction,
                             double chip_distance_meters);

    /**
     * Updates the control parameters for this ChipTactic.
     *
     * @param chip_origin The location where the chip will be taken
     * @param chip_direction The direction the Robot will chip in
     */
    void updateControlParams(const Point& chip_origin, const Point& chip_target);

    void accept(TacticVisitor& visitor) const override;

   private:
    std::unique_ptr<FSM<ChipFSM>> fsm_init() override;

    void updatePrimitive(const TacticUpdate& tactic_update, bool reset_fsm) override;

    // Tactic parameters
    ChipFSMControlParams control_params;
};

// Creates a new tactic called KickoffChipTactic that is a duplicate of ChipTactic
COPY_TACTIC(KickoffChipTactic, ChipTactic)
