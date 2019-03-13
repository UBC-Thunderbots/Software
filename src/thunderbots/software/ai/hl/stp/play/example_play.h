#pragma once

#include "ai/hl/stp/play/play.h"

/**
 * An example Play that moves the robots in a circle around the ball
 */
class ExamplePlay : public Play
{
   public:
    static const std::string name;

    ExamplePlay() = default;

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    std::vector<std::shared_ptr<Tactic>> getNextTactics(TacticCoroutine::push_type &yield,
                                                        const World &world) override;
};
