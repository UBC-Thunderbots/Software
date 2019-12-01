#pragma once

#include "software/multithreading/subject.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/multithreading/threaded_observer.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

// TODO: comment
class WorldStateValidator : public ThreadedObserver<World>, public Subject<World>
{
   public:
    explicit WorldStateValidator();

    virtual ~WorldStateValidator() = default;

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the internal
    // multithreading this class potentially uses
    WorldStateValidator& operator=(const WorldStateValidator&) = delete;
    WorldStateValidator(const WorldStateValidator&)            = delete;

    /**
     *
     *
     * @param validation_functions
     * @param continuous_validation_functions
     * @param timeout
     * @return
     */
    bool waitForValidationToPass(
        const std::vector<ValidationFunction>& validation_functions,
        const std::vector<ValidationFunction>& continuous_validation_functions,
        const Duration& timeout);

   private:
    void onValueReceived(World world) override;

    const Duration world_buffer_timeout = Duration::fromSeconds(5);
    const std::size_t world_buffer_size = 10;
    ThreadSafeBuffer<World> world_buffer;
};
