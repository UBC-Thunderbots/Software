#pragma once

#include "software/ai/hl/hl.h"
#include "software/ai/hl/stp/play_info.h"
#include "software/ai/navigator/navigator.h"
#include "software/time/timestamp.h"
#include "software/world/world.h"

/**
 * This class wraps all our AI logic and decision making.
 */
class AI final
{
   public:
    AI() = delete;

    /**
     * Create an AI with given configurations
     * @param ai_config The AI configuration
     * @param control_config The AI Control configuration
     * @param play_config The Play configuration
     */
    explicit AI(std::shared_ptr<const AiConfig> ai_config,
                std::shared_ptr<const AiControlConfig> control_config,
                std::shared_ptr<const PlayConfig> play_config);

    /**
     * Calculates the Primitives that should be run by our Robots given the current
     * state of the world.
     *
     * @param world The state of the World with which to make the decisions
     *
     * @return the Primitives that should be run by our Robots given the current
     * state of the world.
     */
    std::unique_ptr<TbotsProto::PrimitiveSet> getPrimitives(const World& world) const;

    /**
     * Returns information about the currently running plays and tactics, including the
     * name of the play, and which robots are running which tactics
     *
     * @return information about the currently running plays and tactics
     */
    PlayInfoProto getPlayInfoProto() const;

    std::shared_ptr<Navigator> getNavigator() const;

   private:
    std::shared_ptr<Navigator> navigator;
    std::unique_ptr<HL> high_level;
};
