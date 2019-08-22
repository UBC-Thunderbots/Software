#include "backend/backend_factory.h"

#include <exception>

BackendRegistry& BackendFactory::getMutableRegistry()
{
    static BackendRegistry instance;
    return instance;
}

const BackendRegistry& BackendFactory::getRegistry()
{
    return BackendFactory::getMutableRegistry();
}

std::vector<std::string> BackendFactory::getRegisteredBackendNames()
{
    std::vector<std::string> names;

    for (auto iter = BackendFactory::getRegistry().begin();
         iter != BackendFactory::getRegistry().end(); iter++)
    {
        names.emplace_back(iter->first);
    }
    return names;
}

std::vector<std::function<std::unique_ptr<Backend>()>>
BackendFactory::getRegisteredBackendConstructors()
{
    std::vector<std::function<std::unique_ptr<Backend>()>> constructors;

    for (auto iter = BackendFactory::getRegistry().begin();
         iter != BackendFactory::getRegistry().end(); iter++)
    {
        constructors.emplace_back(iter->second);
    }
    return constructors;
}

void BackendFactory::registerBackend(
    std::string backend_name, std::function<std::unique_ptr<Backend>()> backend_creator)
{
    BackendFactory::getMutableRegistry().insert(
        std::make_pair(backend_name, backend_creator));
}

std::unique_ptr<Backend> BackendFactory::createBackend(const std::string& backend_name)
{
    auto registry = BackendFactory::getRegistry();
    auto iter       = registry.find(backend_name);
    if (iter != registry.end())
    {
        return iter->second();
    }
    else
    {
        std::string msg = std::string("No constructor for '" + backend_name +
                                      "' found in the BackendFactory");
        throw std::invalid_argument(msg);
    }
}
