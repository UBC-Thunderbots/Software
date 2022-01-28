#pragma once

#include "proto/tbots_software_msgs.pb.h"
#include "shared/robot_constants.h"
#include "software/geom/angle.h"
#include "software/geom/angular_velocity.h"
#include "software/geom/point.h"
#include "software/handheld_controller/controller.h"
#include "software/multithreading/first_in_first_out_threaded_observer.h"
#include "software/multithreading/subject.hpp"

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
     * @param robot_constants The robot constants
     */
    explicit ControllerPrimitiveGenerator(
        std::shared_ptr<const HandheldControllerConfig> controller_input_config,
        const RobotConstants_t& robot_constants);

    void onValueReceived(ControllerInput world) override;
    /**
     * Given a ControllerInput, creates a Primitive to implement the desired behavior
     *
     * @param controller_input The ControllerInput to convert to a Primitive
     * @param controller_input_config The config for the PrimitiveGenerator
     * @param robot_constants The robot constants
     *
     * @return A Primitive that will implement the behavior of the given ControllerInput
     */
    static std::unique_ptr<TbotsProto::Primitive> createPrimitiveFromControllerInput(
        const ControllerInput& controller_input,
        std::shared_ptr<const HandheldControllerConfig> controller_input_config,
        const RobotConstants_t& robot_constants);

   private:
    std::shared_ptr<const HandheldControllerConfig> controller_input_config;
    RobotConstants_t robot_constants;
};
