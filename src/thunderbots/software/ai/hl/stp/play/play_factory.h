#pragma once
#include <memory>
#include <string>
#include <unordered_map>

#include "ai/hl/stp/play/play.h"

// A quality of life typedef to make things shorter and more readable
typedef std::unordered_map<std::string, std::function<std::unique_ptr<Play>()>>
    PlayRegistry;

/**
 * The PlayFactory is an Abstract class that provides an interface for Play Factories
 * to follow. This makes it easy to maintain a list of factories and get the corresponding
 * plays through the generic interface.
 */
class PlayFactory
{
   public:
    /**
     * Returns a unique pointer to a newly constructed Play of the given type/name
     *
     * @param play_name The name of the Play to construct. This value must be in the
     * Play registry
     * @throws std::invalid_argument if the given play_name is not found in the Play
     * registry
     *
     * @return a unique pointer to a newly constructed Play of the given type/name
     */
    static std::unique_ptr<Play> createPlay(const std::string& play_name);

    /**
     * Returns a const reference to the Play registry. The registry is a map of Play names
     * to a "create" function that will create and return a unique_ptr to a new concrete
     * instance of the Play.
     *
     * @return a const reference to the Play registry
     */
    static const PlayRegistry& getRegistry();

    /**
     * Returns a list of names of all the existing Plays
     *
     * @return a list of names of all the existing Plays
     */
    static std::vector<std::string> getRegisteredPlayNames();

    /**
     * Returns a list of constructor functions for all the existing Plays
     *
     * @return a list of constructor functions for all the existing Plays
     */
    static std::vector<std::function<std::unique_ptr<Play>()>>
    getRegisteredPlayConstructors();

   protected:
    /**
     * Adds a Play to the Play Registry
     *
     * @param play_name The name of the Play to be added
     * @param play_creator A "create" function that takes no arguments and will return
     * a unique_ptr to a new instance of the specified Play
     */
    static void registerPlay(std::string play_name,
                             std::function<std::unique_ptr<Play>()> play_creator);

   private:
    /**
     * Returns a reference to the Play registry. The registry is a map of Play names
     * to a "create" function that will create and return a unique_ptr to a new concrete
     * instance of the Play, which allows the code to be aware
     * of all the Plays that are available.
     *
     * This is the same as the above public getRegistry function. We need a mutable
     * version in order to add entries to the registry. The function is private so that
     * only this class can make the modifications. Outside sources should not have direct
     * access to modify the registry.
     *
     * @return a mutable reference to the Play registry
     */
    static PlayRegistry& getMutableRegistry();
};

/**
 * This templated play factory class is used by Plays that are derived from the Abstract
 * Play class. Its purpose is to create a Factory for the implemented Play and
 * automatically register the play in the PlayFactory registry.
 *
 * Declaring the static variable will also cause it to be initialized at the start of the
 * program (because it's static). This will immediately call the constructor, which adds
 * the play T to the PlayFactory registry. From then on, the rest of the program
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
        auto play_creator = []() -> std::unique_ptr<Play> {
            return std::make_unique<T>();
        };
        PlayFactory::registerPlay(T::name, play_creator);
    }
};
