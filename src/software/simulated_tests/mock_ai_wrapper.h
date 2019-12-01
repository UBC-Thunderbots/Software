#pragma once

#include "software/ai/primitive/primitive.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/world/world.h"

class MockAIWrapper : public ThreadedObserver<World>,
                      public Subject<ConstPrimitiveVectorPtr>
{
   public:
    explicit MockAIWrapper() = default;

   private:
    void onValueReceived(World world) override;
};