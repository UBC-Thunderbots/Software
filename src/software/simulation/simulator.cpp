#include "software/simulation/simulator.h"

#include "software/backend/output/radio/mrf/mrf_primitive_visitor.h"
#include "software/simulation/simulator_ball_singleton.h"
#include "software/simulation/simulator_robot_singleton.h"
extern "C"
{
#include "firmware/app/world/firmware_ball.h"
#include "firmware/app/world/firmware_robot.h"
#include "firmware/app/world/firmware_world.h"
}

Simulator::Simulator(const World& world) : physics_world(world)
{
    for (auto physics_robot : physics_world.getFriendlyPhysicsRobots())
    {
        auto simulator_robot = std::make_shared<SimulatorRobot>(physics_robot);

        auto firmware_robot = SimulatorRobotSingleton::createFirmwareRobot();
        auto firmware_ball  = SimulatorBallSingleton::createFirmwareBall();
        // Release ownership of the pointers so the firmware_world can take ownership
        FirmwareWorld_t* firmware_world_raw =
            app_firmware_world_create(firmware_robot.release(), firmware_ball.release());
        auto firmware_world =
            std::shared_ptr<FirmwareWorld_t>(firmware_world_raw, FirmwareWorldDeleter());

        simulator_robots.insert(std::make_pair(simulator_robot, firmware_world));
    }

    simulator_ball = std::make_shared<SimulatorBall>(physics_world.getPhysicsBall());
}

void Simulator::stepSimulation(const Duration& time_step)
{
    SimulatorBallSingleton::setSimulatorBall(simulator_ball);
    for (auto& iter : simulator_robots)
    {
        auto simulator_robot = iter.first;
        auto firmware_world  = iter.second;
        SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
        SimulatorRobotSingleton::runPrimitiveOnCurrentSimulatorRobot(firmware_world);
    }

    physics_world.stepSimulation(time_step);
}

void Simulator::setPrimitives(ConstPrimitiveVectorPtr primitives)
{
    if (!primitives)
    {
        return;
    }

    SimulatorBallSingleton::setSimulatorBall(simulator_ball);
    for (const auto& primitive_ptr : *primitives)
    {
        primitive_params_t primitive_params = getPrimitiveParams(primitive_ptr);
        unsigned int primitive_index        = getPrimitiveIndex(primitive_ptr);

        auto simulator_robots_iter =
            std::find_if(simulator_robots.begin(), simulator_robots.end(),
                         [&primitive_ptr](const auto& robot_world_pair) {
                             return robot_world_pair.first->getRobotId() ==
                                    primitive_ptr->getRobotId();
                         });

        if (simulator_robots_iter != simulator_robots.end())
        {
            auto simulator_robot = (*simulator_robots_iter).first;
            auto firmware_world  = (*simulator_robots_iter).second;
            SimulatorRobotSingleton::setSimulatorRobot(simulator_robot);
            SimulatorRobotSingleton::startNewPrimitiveOnCurrentSimulatorRobot(
                firmware_world, primitive_index, primitive_params);
        }
    }
}

World Simulator::getWorld()
{
    return physics_world.getWorld();
}

primitive_params_t Simulator::getPrimitiveParams(
    const std::unique_ptr<Primitive>& primitive)
{
    // The MRFPrimitiveVisitor handles most of the encoding for us
    MRFPrimitiveVisitor mrf_pv;
    primitive->accept(mrf_pv);
    RadioPrimitive_t radio_primitive = mrf_pv.getSerializedRadioPacket();
    primitive_params_t primitive_params;
    std::array<double, 4> param_array = radio_primitive.param_array;
    for (unsigned int i = 0; i < param_array.size(); i++)
    {
        // The data is already scaled appropriately for us from the
        // getSerializedRadioPacket function. We just need to pack it
        // into an int16_t
        double data                = param_array[i];
        primitive_params.params[i] = static_cast<int16_t>(std::round(data));
    }

    primitive_params.slow  = radio_primitive.slow;
    primitive_params.extra = radio_primitive.extra_bits;

    return primitive_params;
}

unsigned int Simulator::getPrimitiveIndex(const std::unique_ptr<Primitive>& primitive)
{
    MRFPrimitiveVisitor mrf_pv;
    primitive->accept(mrf_pv);
    RadioPrimitive_t radio_primitive = mrf_pv.getSerializedRadioPacket();
    auto primitive_index = static_cast<unsigned int>(radio_primitive.prim_type);

    return primitive_index;
}
