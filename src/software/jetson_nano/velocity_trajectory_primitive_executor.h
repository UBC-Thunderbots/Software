#include "proto/primitive.pb.h"
#include "software/geom/vector.h"
#include "software/jetson_nano/primitive_executor.h"
#include "software/world/world.h"

extern "C"
{
#include "firmware/app/control/trajectory_planner.h"
}

/**
 * Executes primitives with a velocity trajectory
 *
 * NOTE: This class is _NOT_ threadsafe
 */
class VelocityTrajectoryPrimitiveExecutor : public PrimitiveExecutor
{
   public:
    /**
     * Start running a primitive
     *
     * @param robot_id The robot id to start/execute the primitive on
     * @param primitive The primitive to start
     */
    void startPrimitive(RobotConstants_t robot_constants, RobotId robot_id,
                        const World& world,
                        std::unique_ptr<TbotsProto::Primitive> primitive) override;

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     *
     * @param world The current world to step the primitive on
     * @returns DirectPerWheelControl The per-wheel direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(
        const World& world) override;

   private:
    std::unique_ptr<TbotsProto::DirectControlPrimitive_DirectVelocityControl>
    planVelocity(const Point& robot_current_position,
                 const Vector& robot_current_velocity,
                 const Angle& robot_current_orientation,
                 const Point& robot_target_position, const float target_spin_rev_per_s,
                 const float max_speed_m_per_s, const float final_speed_m_per_s);

    RobotConstants_t robot_constants_;
    RobotId robot_id_;
    std::unique_ptr<TbotsProto::Primitive> current_primitive_;
    VelocityTrajectory_t velocity_trajectory_;
    PositionTrajectory_t position_trajectory_;
};
