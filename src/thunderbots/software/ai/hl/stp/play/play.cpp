#include "ai/hl/stp/play/play.h"

//const std::vector<std::shared_ptr<PlayFactory>>& Play::getRegistry()
//{
//    return Play::getMutableRegistry();
//}
//
//std::vector<std::shared_ptr<PlayFactory>>& Play::getMutableRegistry()
//{
//    static std::vector<std::shared_ptr<PlayFactory>> instance;
//    return instance;
//}
//
//std::vector<std::string> Play::getPlayNames()
//{
//    std::vector<std::string> names;
//
//    for (const auto& play_factory : Play::getRegistry())
//    {
//        names.emplace_back(play_factory->getInstance()->name());
//    }
//    return names;
//}
//
//void Play::registerPlay(std::shared_ptr<PlayFactory> play_factory)
//{
//    Play::getMutableRegistry().emplace_back(play_factory);
//}
