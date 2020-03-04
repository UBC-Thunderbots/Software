#pragma once

#include "software/multithreading/subject.h"
#include "software/multithreading/thread_safe_buffer.h"
#include "software/multithreading/threaded_observer.h"
#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * This class run ValidationFunctions on each World is receives and is able to report
 * the results. It provides a passthrough for World data (is an Observer for the World
 * and also a Subject, so that it can re-publish the World once it is done processing),
 * so that it can guarantee it has had the change to finish its validation before the rest
 * of the system sees / uses the World.
 */
class WorldStateValidator : public ThreadedObserver<World>, public Subject<World>
{
   public:
    /*
     * Creates a new WorldStateValidator
     */
    explicit WorldStateValidator();

    virtual ~WorldStateValidator() = default;

    // Delete the copy and assignment operators because this class really shouldn't need
    // them and we don't want to risk doing anything nasty with the coroutines
    // contained in and used by this class
    WorldStateValidator& operator=(const WorldStateValidator&) = delete;
    WorldStateValidator(const WorldStateValidator&)            = delete;

    /**
     * Runs the given validation functions on each received World, until all the
     * validation functions pass or the timeout has passed (in World time, NOT wall-clock
     * time). This is a blocking function.
     *
     * @param validation_functions The ValidationFunctions to run. These functions will be
     * run to completion and not restarted. They will report they are "done" once they
     * have completed. These are good for checking certain things happen in a given order,
     * or "stages".
     * @param continuous_validation_functions The continuous validation functions to run.
     * These functions will be continually restarted and run if they finish. They only
     * report failures, and do not have the sense of "completing when done". They are
     * useful for more stateless assertions about the world that must always be true.
     * @param timeout How long to wait (in World time) for the validation functions to
     * pass
     *
     * @return true if all the validation functions have passed before the timeout has
     * been reached, and false otherwise
     */
    bool waitForValidationToPass(
        const std::vector<ValidationFunction>& validation_functions,
        const std::vector<ValidationFunction>& continuous_validation_functions,
        const Duration& timeout);

   private:
    void onValueReceived(World world) override;

    // How long to wait to receive a world, in wall-clock time.
    // This is a somewhat arbitrary value that was chosen to be long enough such that
    // tests won't time-out on slow machines like CI.
    const Duration world_buffer_timeout = Duration::fromSeconds(10);
    // This class makes the assumption that it will finish processing the World before
    // it is re-published to the rest of the system. We know that the AI will wait for a
    // new world before running, and that the SimulatorBackend will wait for new
    // primitives before running, so we can guarantee we will never receive worlds faster
    // than they can be validated, so we only need to buffer a single value
    const std::size_t world_buffer_size = 1;
    ThreadSafeBuffer<World> world_buffer;
};
