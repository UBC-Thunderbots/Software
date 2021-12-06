#pragma once

#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>


#include "Goal.h"
#include "Simulator.h"
#include "Vector2.h"

/**
 * \class  Agent
 * \brief  An agent in the simulation.
 */
class Agent {
private:
    /**
     * \class  Candidate
     * \brief  A candidate point.
     */
    class Candidate {
    public:
        /**
         * \brief  Constructor.
         */
        Candidate() : velocityObstacle1_(0), velocityObstacle2_(0) { }

        /**
         * \brief  The position of the candidate point.
         */
        Vector2 position_;

        /**
         * \brief  The number of the first velocity obstacle.
         */
        int velocityObstacle1_;

        /**
         * \brief  The number of the second velocity obstacle.
         */
        int velocityObstacle2_;
    };

    /**
     * \class  VelocityObstacle
     * \brief  A hybrid reciprocal velocity obstacle.
     */
    class VelocityObstacle {
    public:
        /**
         * \brief  Constructor.
         */
        VelocityObstacle() { }
        /**
         * \brief  The position of the apex of the hybrid reciprocal velocity obstacle.
         */
        Vector2 apex_;

        /**
         * \brief  The direction of the first side of the hybrid reciprocal velocity obstacle.
         */
        Vector2 side1_;

        /**
         * \brief  The direction of the second side of the hybrid reciprocal velocity obstacle.
         */
        Vector2 side2_;
    };

    /**
     * \brief      Constructor.
     * \param[in]  simulator  The simulation.
     */
    explicit Agent(Simulator *simulator);

    /**
     * \brief      Constructor.
     * \param[in]  simulator  The simulation.
     * \param[in]  position   The starting position of this agent.
     * \param[in]  goalNo     The goal number of this agent.
     */
    Agent(Simulator *simulator, const Vector2 &position, std::size_t goalNo);

    /**
     * \brief      Constructor.
     * \param[in]  simulator          The simulation.
     * \param[in]  position           The starting position of this agent.
     * \param[in]  goalNo             The goal number of this agent.
     * \param[in]  neighborDist       The maximum neighbor distance of this agent.
     * \param[in]  maxNeighbors       The maximum neighbor count of this agent.
     * \param[in]  radius             The radius of this agent.
     * \param[in]  goalRadius         The goal radius of this agent.
     * \param[in]  prefSpeed          The preferred speed of this agent.
     * \param[in]  maxSpeed           The maximum speed of this agent.
     * \param[in]  uncertaintyOffset  The uncertainty offset of this agent.
     * \param[in]  maxAccel           The maximum acceleration of this agent.
     * \param[in]  velocity           The initial velocity of this agent.
     * \param[in]  orientation        The initial orientation (in radians) of this agent.
     */
    Agent(Simulator *simulator, const Vector2 &position, std::size_t goalNo, float neighborDist, std::size_t maxNeighbors, float radius, const Vector2 &velocity, float maxAccel, float goalRadius, float prefSpeed, float maxSpeed, float orientation,
          float uncertaintyOffset);

    /**
     * \brief  Computes the neighbors of this agent.
     */
    void computeNeighbors();

    /**
     * \brief  Computes the new velocity of this agent.
     */
    void computeNewVelocity();

    /**
     * \brief  Computes the preferred velocity of this agent.
     */
    void computePreferredVelocity();

    /**
     * \brief          Inserts a neighbor into the set of neighbors of this agent.
     * \param[in]      agentNo  The number of the agent to be inserted.
     * \param[in,out]  rangeSq  The squared range around this agent.
     */
    void insertNeighbor(std::size_t agentNo, float &rangeSq);

    /**
     * \brief  Updates the orientation, position, and velocity of this agent.
     */
    void update();

public: // A
    Simulator *const simulator_;
    Vector2 newVelocity_;
    Vector2 position_;
    Vector2 prefVelocity_;
    Vector2 velocity_;
    std::size_t goalNo_;
    std::size_t maxNeighbors_;
    float goalRadius_;
    float maxAccel_;
    float maxSpeed_;
    float neighborDist_;
    float orientation_;
    float prefSpeed_;
    float radius_;
    float uncertaintyOffset_;
    bool reachedGoal_;
    std::multimap<float, Candidate> candidates_;
    std::set<std::pair<float, std::size_t> > neighbors_;
    std::vector<VelocityObstacle> velocityObstacles_;

    friend class KdTree;
    friend class Simulator;
};
