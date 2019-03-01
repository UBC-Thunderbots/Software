#include "ai/hl/stp/play/play_factory.h"

#include <exception>

PlayRegistry_t& PlayFactory::getMutableRegistry()
{
    static PlayRegistry_t instance;
    return instance;
}

const PlayRegistry_t& PlayFactory::getRegistry()
{
    return PlayFactory::getMutableRegistry();
}

std::vector<std::string> PlayFactory::getRegisteredPlayNames()
{
    std::vector<std::string> names;

    for (auto it = PlayFactory::getRegistry().begin();
         it != PlayFactory::getRegistry().end(); it++)
    {
        names.emplace_back(it->first);
    }
    return names;
}

void PlayFactory::registerPlay(std::string play_name,
                               std::function<std::unique_ptr<Play>()> play_creator)
{
    PlayFactory::getMutableRegistry().insert(std::make_pair(play_name, play_creator));
}

std::unique_ptr<Play> PlayFactory::createPlay(const std::string& play_name)
{
    auto registry = PlayFactory::getRegistry();
    auto it       = registry.find(play_name);
    if (it != registry.end())
    {
        return it->second();
    }
    else
    {
        std::string msg =
            std::string("No constructor for " + play_name + " found in the PlayFactory");
        throw std::invalid_argument(msg);
    }
}
