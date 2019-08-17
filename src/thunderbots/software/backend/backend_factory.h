#pragma once
#include <memory>
#include <string>
#include <unordered_map>

#include "backend/backend.h"

// A quality of life typedef to make things shorter and more readable
typedef std::unordered_map<std::string, std::function<std::unique_ptr<Backend>()>>
        BackendRegistry;

/**
 * The BackendFactory is an Abstract class that provides an interface for Backend Factories
 * to follow. This makes it easy to maintain a list of factories and get the corresponding
 * backends through the generic interface.
 */
class BackendFactory
{
public:
    /**
     * Returns a unique pointer to a newly constructed Backend of the given type/name
     *
     * @param backend_name The name of the Backend to construct. This value must be in the
     * Backend registry
     * @throws std::invalid_argument if the given backend_name is not found in the Backend
     * registry
     *
     * @return a unique pointer to a newly constructed Backend of the given type/name
     */
    static std::unique_ptr<Backend> createBackend(const std::string& backend_name);

    /**
     * Returns a const reference to the Backend registry. The registry is a map of Backend names
     * to a "create" function that will create and return a unique_ptr to a new concrete
     * instance of the Backend.
     *
     * @return a const reference to the Backend registry
     */
    static const BackendRegistry& getRegistry();

    /**
     * Returns a list of names of all the existing Backends
     *
     * @return a list of names of all the existing Backends
     */
    static std::vector<std::string> getRegisteredBackendNames();

    /**
     * Returns a list of constructor functions for all the existing Backends
     *
     * @return a list of constructor functions for all the existing Backends
     */
    static std::vector<std::function<std::unique_ptr<Backend>()>>
    getRegisteredBackendConstructors();

protected:
    /**
     * Adds a Backend to the Backend Registry
     *
     * @param backend_name The name of the Backend to be added
     * @param backend_creator A "create" function that takes no arguments and will return
     * a unique_ptr to a new instance of the specified Backend
     */
    static void registerBackend(std::string backend_name,
                             std::function<std::unique_ptr<Backend>()> backend_creator);

private:
    /**
     * Returns a reference to the Backend registry. The registry is a map of Backend names
     * to a "create" function that will create and return a unique_ptr to a new concrete
     * instance of the Backend, which allows the code to be aware
     * of all the Backends that are available.
     *
     * This is the same as the above public getRegistry function. We need a mutable
     * version in order to add entries to the registry. The function is private so that
     * only this class can make the modifications. Outside sources should not have direct
     * access to modify the registry.
     *
     * @return a mutable reference to the Backend registry
     */
    static BackendRegistry& getMutableRegistry();
};

/**
 * This templated backend factory class is used by Backends that are derived from the Abstract
 * Backend class. Its purpose is to create a Factory for the implemented Backend and
 * automatically register the backend in the BackendFactory registry.
 *
 * Declaring the static variable will also cause it to be initialized at the start of the
 * program (because it's static). This will immediately call the constructor, which adds
 * the backend T to the BackendFactory registry. From then on, the rest of the program
 * can use the registry to find all the Backends that are available (and register with this
 * templated class).
 *
 * @tparam T The class of the Backend to be added to the registry. For example, to add a
 * new class called MoveBackend that inherits from Backend, the following line should be added
 * to the end of the .cpp file (without the quotations):
 * "static TBackendFactory<MoveBackend> factory;"
 */
template <class T>
class TBackendFactory : public BackendFactory
{
    // compile time type checking that T is derived class of Backend
    static_assert(std::is_base_of<Backend, T>::value, "T must be derived class of Backend!");

public:
    TBackendFactory()
    {
        auto backend_creator = []() -> std::unique_ptr<Backend> {
            return std::make_unique<T>();
        };
        BackendFactory::registerBackend(T::name, backend_creator);
    }
};
