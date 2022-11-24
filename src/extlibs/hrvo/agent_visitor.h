#pragma once

#include "agent.h"
#include "linear_velocity_agent.h"
#include "hrvo_agent.h"

class AgentVisitor {
public:
    void visitHRVOAgent(const Agent &agent);
    void visitLinearVelocityAgent(const HRVOAgent &agent);
private:

};