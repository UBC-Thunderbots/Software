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
 * This is how we determine the final result for each pass:
 *
 * Definitely Bad: These are passes that either break an SSL rule
 *                 or passes we just want to avoid completely
 *                 they get a very negative score
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
