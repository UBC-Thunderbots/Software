#include "extlibs/hrvo/linear_velocity_agent.h"

class MotionPlanningSimulatorContext {
public:
    MotionPlanningSimulatorContext(std::vector<HRVOAgent> hrvo_agents, std::vector<LinearVelocityAgent> lv_agents)
            : hrvo_agents(hrvo_agents), lv_agents(lv_agents) {}

    std::vector<HRVOAgent> hrvo_agents;
    std::vector<LinearVelocityAgent> lv_agents;
};
