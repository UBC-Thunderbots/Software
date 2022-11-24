#include "agent_visitor.h"

void AgentVisitor::visitHRVOAgent(const Agent &agent)
{
    std::cout << "visting hrvo" << std::endl
}

void AgentVisitor::visitLinearVelocityAgent(const HRVOAgent &agent)
{
    std::cout << "visting linear" << std::endl
}
