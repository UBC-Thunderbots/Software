#pragma once

#include "software/multithreading/threaded_observer.h"
#include "software/multithreading/subject.h"
#include "software/world/world.h"
#include "software/ai/primitive/primitive.h"

class MockAIWrapper : public ThreadedObserver<World>, public Subject<ConstPrimitiveVectorPtr> {
public:
    explicit MockAIWrapper() = default;

private:
    void onValueReceived(World world) override;
};