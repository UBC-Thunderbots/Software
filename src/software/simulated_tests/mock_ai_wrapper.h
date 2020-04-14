#pragma once

#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/primitive/primitive.h"
#include "software/world/world.h"

/**
 * This is a simple class that Mocks being an AIWrapper. It removes all gameplay logic
 * and simply publishes an empty list of Primitives every time it receives a World.
 */
class MockAIWrapper : public ThreadedObserver<World>,
                      public Subject<ConstPrimitiveVectorPtr>
{
   public:
    explicit MockAIWrapper() = default;

   private:
    void onValueReceived(World world) override;
};
