#include "proto/primitive.pb.h"
#include "software/geom/vector.h"
#include "software/world/world.h"

/**
 * Executes primitives with a velocity trajectory
 *
 * NOTE: This class is _NOT_ threadsafe
 */
class PrimitiveExecutor
{
   public:
    /**
     * Start running a primitive
     *
     * @param robot The robot to start/execute the primitive on
     * @param primitive The primitive to start
     */
    void startPrimitive(const Robot& robot,
                        std::unique_ptr<TbotsProto::Primitive> primitive);

    /**
     * Steps the current primitive and returns a direct control primitive with the
     * target wheel velocities
     *
     * @param robot The current robot to step the primitive on
     * @returns DirectPerWheelControl The per-wheel direct control primitive msg
     */
    std::unique_ptr<TbotsProto::DirectControlPrimitive> stepPrimitive(const Robot& robot);

   private:
    /*
     * Compute the next target linear velocity the robot should be at
     * assuming max acceleration.
     *
     * @param robot The robot we are planning the current primitive for
     */
    Vector getTargetLinearVelocity(const Robot& robot);

    /*
     * Compute the next target angular velocity the robot should be at
     * assuming max acceleration.
     *
     * @param robot The robot we are planning the current primitive for
     */
    AngularVelocity getTargetAngularVelocity(const Robot& robot);

    std::unique_ptr<TbotsProto::Primitive> current_primitive_;
    unsigned num_elements_;
};
