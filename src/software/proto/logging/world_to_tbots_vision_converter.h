#pragma once
#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/world/world.h"

class WorldToTbotsVisionConverter : public Subject<TbotsProto::Vision>,
                                    public ThreadedObserver<World>
{
   public:
    WorldToTbotsVisionConverter() : ThreadedObserver<World>(1000) {}

   private:
    void onValueReceived(World world) override;
};