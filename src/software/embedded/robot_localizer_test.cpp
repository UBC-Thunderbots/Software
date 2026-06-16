#include "software/embedded/robot_localizer.h"

#include <gtest/gtest.h>

#include <cmath>

#include "shared/constants.h"
#include "software/physics/velocity_conversion_util.h"

namespace
{
// Mirror the values thunderloop constructs the localizer with (DivB constants).
RobotLocalizer::RobotLocalizerConfig makeConfig()
{
    return RobotLocalizer::RobotLocalizerConfig{/*process_noise_variance=*/1.0,
                                                /*vision_noise_variance=*/0.01 * 0.01,
                                                /*motor_sensor_noise_variance=*/0.5};
}

constexpr double LOOP_HZ = 300.0;
constexpr double DT      = 1.0 / LOOP_HZ;

// Drives the localizer through one second of a robot moving straight along global +X at
// a constant 1 m/s (heading along +X, not rotating), feeding it the same inputs
// thunderloop does each tick. If feed_vision is false, only the initial vision seed is
// provided (isolates whether the periodic vision fix corrupts the velocity estimate).
RobotLocalizer runConstantVelocity(bool feed_vision, double vision_age = RTT_S / 2)
{
    RobotLocalizer localizer(makeConfig());

    const Vector true_velocity(1.0, 0.0);
    const Angle true_orientation = Angle::zero();
    Point true_position(0.0, 0.0);

    localizer.update(
        RobotLocalizer::VisionData{true_position, true_orientation, RTT_S / 2});

    for (int tick = 0; tick < static_cast<int>(LOOP_HZ); ++tick)
    {
        true_position += true_velocity * DT;

        localizer.update(RobotLocalizer::ImuData{AngularVelocity::zero()});

        const Vector local_velocity =
            globalToLocalVelocity(true_velocity, true_orientation);
        localizer.update(RobotLocalizer::MotorData{
            localToGlobalVelocity(local_velocity, localizer.getOrientation()),
            AngularVelocity::zero()});

        localizer.step(Vector(0.0, 0.0));

        // Periodic vision fix (~60 Hz). Feed the position from RTT_S/2 ago, consistent
        // with the reported age.
        if (feed_vision && tick % 5 == 0)
        {
            localizer.update(
                RobotLocalizer::VisionData{true_position - true_velocity * vision_age,
                                           true_orientation, vision_age});
        }
    }

    return localizer;
}
}  // namespace

// Regression test mirroring the (fixed) thunderloop wiring: orientation, velocity, and
// position should all track a constant-velocity robot.
TEST(RobotLocalizer, tracks_constant_forward_velocity)
{
    const RobotLocalizer localizer = runConstantVelocity(/*feed_vision=*/true);

    std::cerr << "[motor+vision] pos=(" << localizer.getPosition().x() << ", "
              << localizer.getPosition().y() << ") vel=(" << localizer.getVelocity().x()
              << ", " << localizer.getVelocity().y()
              << ") orient=" << localizer.getOrientation().toDegrees() << "deg\n";

    EXPECT_NEAR(localizer.getOrientation().toDegrees(), 0.0, 10.0);
    EXPECT_NEAR(localizer.getVelocity().x(), 1.0, 0.2)
        << "Forward velocity estimate does not track";
    EXPECT_NEAR(localizer.getVelocity().y(), 0.0, 0.2);
    EXPECT_NEAR(localizer.getPosition().x(), 1.0, 0.2);
    EXPECT_NEAR(localizer.getPosition().y(), 0.0, 0.2);
}

// Diagnostic: with no periodic vision fix, the velocity estimate comes purely from the
// motor measurements (a constant 1 m/s). If this tracks but the test above does not,
// the periodic vision update is what corrupts the velocity estimate.
TEST(RobotLocalizer, velocity_tracks_from_motors_without_vision)
{
    const RobotLocalizer localizer = runConstantVelocity(/*feed_vision=*/false);

    std::cerr << "[motor only]   vel=(" << localizer.getVelocity().x() << ", "
              << localizer.getVelocity().y() << ")\n";

    EXPECT_NEAR(localizer.getVelocity().x(), 1.0, 0.2);
    EXPECT_NEAR(localizer.getVelocity().y(), 0.0, 0.2);
}

// Diagnostic: feed vision with a near-zero age, which takes the non-rollback path
// (apply vision to the current state, clear history). If velocity tracks here but not
// with a realistic age, the rollback/replay machinery is the culprit; if it's still
// wrong, the vision measurement's position->velocity covariance coupling is.
TEST(RobotLocalizer, velocity_with_zero_age_vision)
{
    const RobotLocalizer localizer =
        runConstantVelocity(/*feed_vision=*/true, /*vision_age=*/1e-6);

    std::cerr << "[zero-age vision] vel=(" << localizer.getVelocity().x() << ", "
              << localizer.getVelocity().y() << ")\n";

    EXPECT_NEAR(localizer.getVelocity().x(), 1.0, 0.2);
    EXPECT_NEAR(localizer.getVelocity().y(), 0.0, 0.2);
}
