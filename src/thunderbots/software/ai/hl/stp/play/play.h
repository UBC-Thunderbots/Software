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
    /**
     * Returns a pointer to the Play constructed by this Factory
     *
     * @return a shared pointer to the Play constructed by this Factory
     */
    virtual std::shared_ptr<Play> getInstance() = 0;

    virtual ~PlayFactory() = default;
};

/**
 * In the STP framework, a Play is essentially a collection of tactics that represent some
 * "team-wide" goal. It can be thought of like a traditional play in soccer, such as an
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
     * Returns whether or not this Play can be started. For example, the Enemy Team
     * must have the ball for a defensive play to be applicable.
     *
     * @param world The current state of the world
     * @return true if this Play can be started, and false otherwise
     */
    virtual bool isApplicable(const World& world) = 0;

    /**
     * Returns whether or not the invariant for this Play holds (is true). The invariant
     * is a set of conditions that must remain true for the Play to continue running
     * (not necessarily the same as the Applicable conditions). For example, the Invariant
     * for an Offensive Play might be that our team must have possession of the ball. If
     * our team no longer has possession of the ball (ie. the invariant no longer holds),
     * then we should probably run a different Play.
     *
     * @param world The current state of the world
     * @return true if this Play's invariant holds (is true), and false otherwise
     */
    virtual bool invariantHolds(const World& world) = 0;

    /**
     * Returns whether or not the Play has failed. This is kept as a separate condition
     * from the Invariant because it is easier to reason about. For example, a Play that
     * is supposed to retrieve a loose ball would fail if the enemy team got possession
     * of the ball
     *
     * @param world The current state of the world
     * @return true if this Play has failed, and false otherwise
     */
    virtual bool hasFailed(const World& world) = 0;

    /**
     * Gets the tactics that make up this Play. The tactics are returned in decreasing
     * order of priority (the first tactic has the highest priority, the last one has the
     * least). This means that if there are not enough available robots to run all of the
     * tactics, the tactics with the least priority will not be assigned/run
     *
     * @param world The current state of the world
     * @return A list of tactics that should be run as part of this Play, in decreasing
     * order of priority
     */
    virtual std::vector<std::unique_ptr<Tactic>> getTactics(const World& world) = 0;

    /**
     * Returns the name of this Play
     *
     * @return the name of this Play
     */
    virtual std::string name() = 0;

    virtual ~Play() = default;

    /** Factory Stuff **/
    /**
     * Returns a reference to the Play registry. The registry is a list of pointers
     * to all the factories for the existing Play, which allows the code to be aware
     * of all the Plays that are available.
     *
     * @return a const reference to the Play registry
     */
    static const std::vector<std::shared_ptr<PlayFactory>>& getRegistry();

    /**
     * Adds a Play to the Play Registry by adding the corresponding Play Factory
     * @param play_factory A Pointer to the Play Factory to be added
     */
    static void registerPlay(std::shared_ptr<PlayFactory> play_factory);

    /**
     * Returns a list of names of all the existing Plays
     * @return a list of names of all the existing Plays
     */
    static std::vector<std::string> getPlayNames();

   private:
    /**
     * Returns a reference to the Play registry. The registry is a list of pointers
     * to all the factories for the existing Play, which allows the code to be aware
     * of all the Plays that are available.
     *
     * This is the same as the above public getRegistry function. We need a mutable
     * version in order to add entries to the registry. The function is private so that
     * only this class can make the modifications. Outside sources should not have direct
     * access to modify the registry.
     *
     * @return a mutable reference to the Play registry
     */
    static std::vector<std::shared_ptr<PlayFactory>>& getMutableRegistry();
};

/**
 * This templated play factory class is used by Plays that are derived from the Abstract
 * Play class above. Its purpose is to create a Factory for the implemented Play and
 * automatically register the play in the play registry.
 *
 * Declaring the static variable will also cause it to be initialized at the start of the
 * program (because it's static). This will immediately call the constructor, which adds
 * a pointer to the Factory to the Play registry. From then on, the rest of the program
 * can use the registry to find all the Plays that are available (and register with this
 * templated class).
 *
 * @tparam T The class of the Play to be added to the registry. For example, to add a
 * new class called MovePlay that inherits from Play, the following line should be added
 * to the end of the .cpp file (without the quotations):
 * "static TPlayFactory<MovePlay> factory;"
 */
template <class T>
class TPlayFactory : public PlayFactory
{
    // compile time type checking that T is derived class of Play
    static_assert(std::is_base_of<Play, T>::value, "T must be derived class of Play!");

   public:
    TPlayFactory()
    {
        Play::registerPlay(std::make_shared<TPlayFactory>(*this));
    }

    std::shared_ptr<Play> getInstance() override
    {
        return std::make_shared<T>();
    }
};
