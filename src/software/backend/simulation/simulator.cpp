#include "software/backend/simulation/simulator.h"

#include "software/backend/output/radio/mrf/mrf_primitive_visitor.h"
#include "software/backend/simulation/simulator_ball_singleton.h"
#include "software/backend/simulation/simulator_robot_singleton.h"
extern "C"
{
#include "firmware/main/app/world/firmware_ball.h"
#include "firmware/main/app/world/firmware_robot.h"
#include "firmware/main/app/world/firmware_world.h"
}

Simulator::Simulator(const World& world) : physics_world(world)
{
    for (auto physics_robot : physics_world.getFriendlyPhysicsRobots())
    {
        auto simulator_robot = std::make_shared<SimulatorRobot>(physics_robot);
        std::unique_ptr<PrimitiveManager, PrimitiveManagerDeleter> primitive_manager(
            app_primitive_manager_create(), PrimitiveManagerDeleter());
        this->simulator_robots.emplace(simulator_robot, std::move(primitive_manager));
    }
    simulator_ball = std::make_shared<SimulatorBall>(physics_world.getPhysicsBall());

    auto firmware_robot = SimulatorRobotSingleton::createFirmwareRobot();
    auto firmware_ball  = SimulatorBallSingleton::createFirmwareBall();
    // Make sure to release ownership from the local variables so the firmware_world
    // can take ownership
    FirmwareWorld_t* firmware_world_raw =
        app_firmware_world_create(firmware_robot.release(), firmware_ball.release());
    firmware_world = std::unique_ptr<FirmwareWorld_t, FirmwareWorldDeleter>(
        firmware_world_raw, FirmwareWorldDeleter());
}

void Simulator::stepSimulation(const Duration& time_step)
{
    SimulatorBallSingleton::setSimulatorBall(simulator_ball);
    for (auto iter = simulator_robots.begin(); iter != simulator_robots.end(); iter++)
    {
        auto simulator_robot    = iter->first;
        auto& primitive_manager = iter->second;
        SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
        app_primitive_manager_runCurrentPrimitive(primitive_manager.get(),
                                                  firmware_world.get());
    }

    physics_world.stepSimulation(time_step);
}

void Simulator::setPrimitives(ConstPrimitiveVectorPtr primitives)
{
    if (primitives)
    {
        auto& primitive_vector = *primitives;
        for (auto iter = simulator_robots.begin(); iter != simulator_robots.end(); iter++)
        {
            auto simulator_robot                        = iter->first;
            PrimitiveManager* primitive_manager_raw_ptr = iter->second.get();

            auto primitive_iter =
                std::find_if(primitive_vector.begin(), primitive_vector.end(),
                             [&simulator_robot](const auto& p) {
                                 return simulator_robot->getRobotId() == p->getRobotId();
                             });
            if (primitive_iter != primitive_vector.end())
            {
                // Get params and start primitive

                // The MRFPrimitiveVisitor handles most of the encoding for us
                MRFPrimitiveVisitor mrf_pv;
                (*primitive_iter)->accept(mrf_pv);
                RadioPrimitive_t radio_primitive = mrf_pv.getSerializedRadioPacket();
                unsigned int primitive_index =
                    static_cast<unsigned int>(radio_primitive.prim_type);
                primitive_params_t primitive_params;
                std::array<double, 4> param_array = radio_primitive.param_array;
                for (unsigned int i = 0; i < param_array.size(); i++)
                {
                    // The data is already scaled appropriately for us from the
                    // getSerializedRadioPacket function. We just need to pack it
                    // into an in16_t
                    double data                = param_array[i];
                    primitive_params.params[i] = static_cast<int16_t>(std::round(data));
                }

                primitive_params.slow  = radio_primitive.slow;
                primitive_params.extra = radio_primitive.extra_bits;

                SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
                app_primitive_manager_startNewPrimitive(
                    primitive_manager_raw_ptr, firmware_world.get(), primitive_index,
                    &primitive_params);
            }
        }
    }
}

World Simulator::getWorld()
{
    return physics_world.getWorld();
}
