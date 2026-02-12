#pragma once
#include <zlib.h>

#include "pass.h"
#include "software/ai/passing/pass.h"
#include "software/logger/generic_logger.h"
#include "software/world/world.h"

//
// Created by thunderbots on 2/9/26.
//

/*
 * This class is responsible for collecting the features and the results
 * for each pass considered by the Pass Generator
 *
 * These will tell the model what a "bad" pass is:
 * We can't make any determinations on if the pass is good, since
 * we don't know the game state outcome yet. But, we know which
 * passes we'd like to exclude / weigh down
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

 *                 pass speed == 0 or no friendly robots
 *
 * Slightly Bad: These are passes that are considered suboptimal by the
 *               static sigmoid-based rating system. they get a
 *               flat score of -1
 *               open angle to goal too low?
 *
 * Everything else gets a score of 0, so a neutral pass.
 *
 *
 */
class PassFeatureCollector
{
   public:
    PassFeatureCollector(const std::string& log_dir);

    ~PassFeatureCollector();

    void logPassFeatures(const Pass& pass, const World& world,
                         TbotsProto::PassingConfig passing_config) const;

   private:

    static int getPassScore(const Pass& pass, const World& world,
                            TbotsProto::PassingConfig passing_config);

    static std::string createDatasetEntry(const Pass& pass, const World& world,
                                          int score);
    static std::string createPointFeatureEntry(const Point& point);
    static std::string createTeamFeatureEntry(const Team& team);

    std::optional<GenericLogger> logger_;

    static constexpr int DEFINITELY_BAD_SCORE    = -10;
    static constexpr int SLIGHTLY_BAD_SCORE      = -1;
    static constexpr int VERY_SLIGHTLY_BAD_SCORE = -0.1;
    static constexpr int NEUTRAL_SCORE           = 0;
};
