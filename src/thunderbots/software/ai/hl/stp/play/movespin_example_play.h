#pragma once

#include "ai/hl/stp/play/play.h"

/**
 * An example Play that moves the robots in a circle around the ball
 */
class MoveSpinExamplePlay : public Play
{
public:
    static const std::string name;

    MoveSpinExamplePlay() = default;

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield) override;
};
