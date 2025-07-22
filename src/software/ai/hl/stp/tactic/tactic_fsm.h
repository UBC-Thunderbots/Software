#pragma once

#include <functional>
#include <memory>

#include "proto/primitive/primitive_msg_factory.h"
#include "proto/parameters.pb.h"
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

/*
 * A general FSM class with some utilities.
 * T_FSM_CTP should be a structure containing the control parameters for the FSM being built.
 */
template<class T_FSM_CTP>
class TacticFSM
{
public:
    /**
     * The Update struct is the only event that a tactic fsm should respond to and it is
     * composed of the following structs:
     *
     * T_FSM_CTP - uniquely defined by each tactic to control the FSM
     * TacticUpdate - common struct that contains Robot, World, and SetPrimitiveCallback
     */
    struct Update
    {
        Update(const T_FSM_CTP &control_params, const TacticUpdate &common)
                : control_params(control_params), common(common)
        {
        }
        T_FSM_CTP control_params;
        TacticUpdate common;
    };

    explicit TacticFSM(std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr): ai_config_ptr(ai_config_ptr){}

protected:
    // Former constructors took what they needed from ai_config and stored it locally.
    // Now, we store ai_config as a pointer and use it to update as needed.
    std::shared_ptr<TbotsProto::AiConfig> ai_config_ptr;
};



#define SET_STOP_PRIMITIVE_ACTION                                                        \
    [this](auto event) { event.common.set_primitive(std::make_unique<StopPrimitive>()); }
