#pragma once

#include "software/handheld_controller/controller.h"
#include "software/multithreading/subject.h"
#include "software/multithreading/threaded_observer.h"
#include "software/primitive/primitive.h"

/**
 * An observer that converts ControllerInputs into Primitives
 */
class ControllerPrimitiveGenerator : public ThreadedObserver<ControllerInput>,
                                     public Subject<ConstPrimitiveVectorPtr>
{
   public:
    /**
     * Creates a new PrimitiveGenerator
     *
     * @param controller_input_config The config for the PrimitiveGenerator
     */
    explicit ControllerPrimitiveGenerator(
        std::shared_ptr<const HandheldControllerInputConfig> controller_input_config);

    void onValueReceived(ControllerInput world) override;

   private:
    /**
     * Given a ControllerInput, creates a Primitive to implement the desired behavior
     *
     * @param controller_input The ControllerInput to convert to a Primitive
     *
     * @return A Primitive that will implement the behavior of the given ControllerInput
     */
    std::unique_ptr<Primitive> createPrimitiveFromControllerInput(
        const ControllerInput& controller_input);

    std::shared_ptr<const HandheldControllerInputConfig> controller_input_config;
};
