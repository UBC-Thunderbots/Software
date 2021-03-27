#include "software/simulation/simulator_robot.h"

SimulatorRobot::SimulatorRobot()
    : autokick_speed_m_per_s(std::nullopt), autochip_distance_m(std::nullopt)
{
    primitive_manager =
        std::unique_ptr<PrimitiveManager, FirmwarePrimitiveManagerDeleter>(
            app_primitive_manager_create(), FirmwarePrimitiveManagerDeleter());
}
