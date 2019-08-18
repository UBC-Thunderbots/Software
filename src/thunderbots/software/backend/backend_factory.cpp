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

    for (auto it = BackendFactory::getRegistry().begin();
         it != BackendFactory::getRegistry().end(); it++)
    {
        names.emplace_back(it->first);
    }
    return names;
}

std::vector<std::function<std::unique_ptr<Backend>()>>
BackendFactory::getRegisteredBackendConstructors()
{
    std::vector<std::function<std::unique_ptr<Backend>()>> constructors;

    for (auto it = BackendFactory::getRegistry().begin();
         it != BackendFactory::getRegistry().end(); it++)
    {
        constructors.emplace_back(it->second);
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
    auto it       = registry.find(backend_name);
    if (it != registry.end())
    {
        return it->second();
    }
    else
    {
        std::string msg = std::string("No constructor for '" + backend_name +
                                      "' found in the BackendFactory");
        throw std::invalid_argument(msg);
    }
}
