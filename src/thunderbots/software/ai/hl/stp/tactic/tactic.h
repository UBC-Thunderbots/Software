#pragma once

#include <memory>
#include "ai/intent/intent.h"
#include "ai/world/world.h"


class Tactic
{
   public:
    virtual std::unique_ptr<Intent> getNextIntent(const World& world,
                                                  const Robot& robot) = 0;

    virtual std::pair<Robot, Team> selectRobot(const Team& available_robots) = 0;

   private:
};
