#pragma once

#include "shared/proto/tbots_software_msgs.pb.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"
#include "software/handheld_controller/controller.h"
#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/multithreading/subject.h"

/**
 * An observer that converts ControllerInputs into Primitives
 */
class ControllerPrimitiveGenerator
    : public FirstInFirstOutThreadedObserver<ControllerInput>,
      public Subject<TbotsProto::PrimitiveSet>
{
   public:
    /**
     * Creates a new PrimitiveGenerator
     *
     * @param controller_input_config The config for the PrimitiveGenerator
     */
    explicit ControllerPrimitiveGenerator(
        std::shared_ptr<const HandheldControllerConfig> controller_input_config);

    void onValueReceived(ControllerInput world) override;

    /**
     * Creates a new DirectControl Primitive AI could output this primitive to control the
     * linear velocity, angular velocity, and dribbler speed of a specific robot
     *
     * @param velocity x/y velocity vector
     * @param angular_velocity The angular velocity
     * @param dribbler_rpm The dribbler speed in rpm
     *
     * @return Pointer to the DirectControl Primitive
     */
    static std::unique_ptr<TbotsProto::Primitive> createDirectControlPrimitive(
        const Vector& velocity, AngularVelocity angular_velocity, double dribbler_rpm);

    /**
     * Given a ControllerInput, creates a Primitive to implement the desired behavior
     *
     * @param controller_input The ControllerInput to convert to a Primitive
     *
     * @return A Primitive that will implement the behavior of the given ControllerInput
     */
    static std::unique_ptr<TbotsProto::Primitive> createPrimitiveFromControllerInput(
        const ControllerInput& controller_input,
        std::shared_ptr<const HandheldControllerConfig> controller_input_config);

   private:
    std::shared_ptr<const HandheldControllerConfig> controller_input_config;
};
