#pragma once

#include <memory>
#include <vector>
#include "ai/hl/stp/tactic/tactic.h"
#include "ai/world/world.h"

// Forward declaration of the Play class so the PlayFactory can reference it.
class Play;

/** Note **/
// We keep the PlayFactory and Play classes in the same file to avoid cyclic includes
// (where each file includes the other), since it causes compilation to fail

/**
 * The PlayFactory is an Abstract class that provides an interface for Play Factories
 * to follow. This makes it easy to maintain a list of factories and get the corresponding
 * plays through the generic interface.
 */
class PlayFactory
{
   public:
    virtual std::shared_ptr<Play> getInstance() = 0;
};

/**
 * In the STP framework, a Play is essentially a collection of tactics that represent some
 * "team-wide" goal. It can be thought of like a tradition play in soccer, such as an
 * offensive play, defensive play, or a specific play used for corner kicks.
 *
 * When we are running autonomously, different Plays are selected at different times
 * based on the state of the world, and this switching of Plays is what allows our AI to
 * play a full game of soccer, as long a we provide a Play for any given scenario.
 */
class Play
{
   public:
    /**
     * Returns whether or not this Play can be started.
     *
     * @param world The current state of the world
     * @return true if this Play can be started, and false otherwise
     */
    virtual bool isApplicable(const World& world) = 0;

    virtual bool invariantHolds(const World& world) = 0;
    virtual bool hasFailed(const World& world)      = 0;

    virtual std::vector<std::unique_ptr<Tactic>> getTactics(const World& world) = 0;

    virtual std::string name() = 0;

    static std::vector<std::string> getPlayNames();

    // Factory stuff
    static const std::vector<std::shared_ptr<PlayFactory>>& getRegistry();
    static void registerPlay(std::shared_ptr<PlayFactory> play_factory);

   private:
    static std::vector<std::shared_ptr<PlayFactory>>& getMutableRegistry();
};

#define REGISTER_PLAY(klass)                                                             \
    class klass##Factory : public PlayFactory                                            \
    {                                                                                    \
       public:                                                                           \
        klass##Factory()                                                                 \
        {                                                                                \
            Play::registerPlay(std::shared_ptr<klass##Factory>(this));                   \
        }                                                                                \
        std::shared_ptr<Play> getInstance() override                                     \
        {                                                                                \
            return std::make_shared<klass>();                                            \
        }                                                                                \
    };                                                                                   \
    static klass##Factory global_##klass##Factory;
