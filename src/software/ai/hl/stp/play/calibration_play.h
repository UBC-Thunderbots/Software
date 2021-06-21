#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/hl/stp/play/play.h"

/**
 * A calibration play that performs a sequence of operations and outputs
 * calibration values on the field.
 */
class CalibrationPlay : public Play
{
   public:
    explicit CalibrationPlay(std::shared_ptr<const PlayConfig> config);

    /**
     * This function returns whether the play is applicable
     * Applicable describes the conditions that must be met for the play to start
     * For instance, the play should start when it is kick-off
     *
     * @param world The state of the world with which to perform the evaluation
     * @return If ExamplePlay is applicable
     */
    bool isApplicable(const World &world) const override;

    /**
     * This function returns whether the play's invariant holds
     * Invariant describes the conditions that must be met for the play to continue
     * running For instance, the invariant may not hold if we lose possession of the ball
     *
     * @param world The state of the world with which to perform the evaluation
     * @return If ExamplePlay's invariant holds
     */
    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;
};
