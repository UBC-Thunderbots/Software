#pragma once
#include "pass.h"
#include "software/ai/passing/pass.h"
#include "software/world/world.h"

//
// Created by thunderbots on 2/9/26.
//

/*
 * This class is responsible
 *
 * These are the features we want to collect:
 *
 * Pass Receive Position x and y
 * Ball Position x and y
 * Positions of friendly and enemy robots, x and y
 * Closest distance from pass receive point to any enemy
 * Time taken by closest friendly to reach ball
 * Time taken by closest friendly to turn to receive ball
 * enemy intercept time somehow?
 * Open angle to goal after the pass
 *
 *
 * This is how we determine the final result for each pass:
 *
 * Definitely Bad: These are passes that either break an SSL rule
 *                 or passes we just want to avoid completely
 *                 they get a very negative score
 *
 *                 passes too close to passer
 *                 passes in enemy defense area
 *                 passes too close to field edges
 *                 pass speed == 0 or no friendly robots
 *
 * Slightly Bad: These are passes that are considered suboptimal by the
 *               static sigmoid-based rating system
 *
 *               time to receive / time to turn > config val
 *               too long of a backwards pass
 *               too close to enemy?
 *               enemy intercept risk?
 *               open angle to goal too low?
 *
 *
 */
class PassFeatureCollector
{
   public:
    PassFeatureCollector(const std::string& log_path, bool friendly_colour_yellow);

    ~PassFeatureCollector();

    void logPassFeatures(const Pass& pass, const World& world);

   private:
    void logFeaturesToFile(std::string log_entry);
    void isPassDefinitelyBad(const Pass& pass, const World& world);

    std::string log_dir_;
    std::string log_path_;
    bool friendly_colour_yellow_{};
};
