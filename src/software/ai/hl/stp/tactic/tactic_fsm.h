#pragma once

#include <functional>
#include <memory>

#include "proto/parameters.pb.h"
#include "proto/primitive/primitive_msg_factory.h"
#include "proto/tbots_software_msgs.pb.h"
#include "software/ai/hl/stp/tactic/primitive.h"
#include "software/ai/hl/stp/tactic/stop_primitive.h"
#include "software/util/sml_fsm/sml_fsm.h"
#include "software/world/world.h"

using SetPrimitiveCallback = std::function<void(std::shared_ptr<Primitive>)>;

// The tactic update struct is used to update tactics and set the new primitive
struct TacticUpdate
{
    TacticUpdate(const Robot &robot, const WorldPtr &world_ptr,
                 const SetPrimitiveCallback &set_primitive_fun)
        : robot(robot), world_ptr(world_ptr), set_primitive(set_primitive_fun)
    {
    }

    // updated robot that tactic is assigned to
    Robot robot;
    // updated world
    WorldPtr world_ptr;
    // callback to return the next primitive
    SetPrimitiveCallback set_primitive;
};

/**
 * A general FSM class with some utilities for tactics.
 *
 * @tparam TFsm The Tactic FSM that inherits from an instance of this template.
 * See https://en.wikipedia.org/wiki/Curiously_recurring_template_pattern
 */
template <class TFsm>
class TacticFSM
{
   public:
    using ControlParams = TFsm::ControlParams;
    /**
     * The Update struct is the only event that a tactic FSM should respond to and it is
     * composed of the following structs:
     *
     * control_params- uniquely defined parameters for each FSM
     * common - common struct that contains Robot, World, and SetPrimitiveCallback
     */
    struct Update
    {
        Update(const ControlParams &control_params, const TacticUpdate &common)
            : control_params(control_params), common(common)
        {
        }
        ControlParams control_params;
        TacticUpdate common;
    };

    explicit TacticFSM(std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr)
        : ai_config_ptr(ai_config_ptr)
    {
    }

   protected:
    // A shared pointer to the ai configuration to configure ai behaviour, shared by all
    // Plays, Tactics, and FSMs
    std::shared_ptr<const TbotsProto::AiConfig> ai_config_ptr;
};

#define SET_STOP_PRIMITIVE_ACTION                                                        \
    [this](auto event) { event.common.set_primitive(std::make_unique<StopPrimitive>()); }
