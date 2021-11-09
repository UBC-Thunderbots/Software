#include "proto/primitive.pb.h"
#include "software/world/world.h"

/**
 * The primitive executor steps the primitive and returns a DirectControlPrimitive
 */
class PrimitiveExecutor
{
   public:
    /**
     * Start running a primitive
     *
     * @param robot_id The robot id to start/execute the primitive on
     * @param primitive The primitive to start
     */
    virtual void startPrimitive(RobotConstants_t robot_constants, RobotId robot_id,
                                const World& world,
                                std::unique_ptr<TbotsProto::Primitive> primitive) = 0;

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     */
    virtual std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(
        const World& world) = 0;
};
