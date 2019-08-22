#include "ai/hl/stp/play/play_factory.h"

#include <exception>

PlayRegistry& PlayFactory::getMutableRegistry()
{
    static PlayRegistry instance;
    return instance;
}

const PlayRegistry& PlayFactory::getRegistry()
{
    return PlayFactory::getMutableRegistry();
}

std::vector<std::string> PlayFactory::getRegisteredPlayNames()
{
    std::vector<std::string> names;

    for (auto iter = PlayFactory::getRegistry().begin();
         iter != PlayFactory::getRegistry().end(); iter++)
    {
        names.emplace_back(iter->first);
    }
    return names;
}

std::vector<std::function<std::unique_ptr<Play>()>>
PlayFactory::getRegisteredPlayConstructors()
{
    std::vector<std::function<std::unique_ptr<Play>()>> constructors;

    for (auto iter = PlayFactory::getRegistry().begin();
         iter != PlayFactory::getRegistry().end(); iter++)
    {
        constructors.emplace_back(iter->second);
    }
    return constructors;
}

void PlayFactory::registerPlay(std::string play_name,
                               std::function<std::unique_ptr<Play>()> play_creator)
{
    PlayFactory::getMutableRegistry().insert(std::make_pair(play_name, play_creator));
}

std::unique_ptr<Play> PlayFactory::createPlay(const std::string& play_name)
{
    auto registry = PlayFactory::getRegistry();
    auto iter       = registry.find(play_name);
    if (iter != registry.end())
    {
        return iter->second();
    }
    else
    {
        std::string msg = std::string("No constructor for '" + play_name +
                                      "' found in the PlayFactory");
        throw std::invalid_argument(msg);
    }
}
