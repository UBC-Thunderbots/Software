#pragma once

#include "ai/hl/stp/tactic/tactic.h"
#include "ai/intent/move_intent.h"
#include "geom/point.h"

class MoveTactic : public Tactic
{
   public:
    explicit MoveTactic(const Point& destination);

    std::unique_ptr<Intent> getNextIntent(const World& world,
                                          const Robot& robot) override;
    Robot selectRobot(const World& world,
                      const std::vector<Robot>& available_robots) override;

   private:
    Point destination;
};
