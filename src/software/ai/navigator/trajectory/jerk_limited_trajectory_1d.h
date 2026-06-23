#pragma once

#include <utility>
#include <vector>

#include "software/ai/navigator/trajectory/trajectory.hpp"

class JerkLimitedTrajectory1D : public Trajectory<double, double, double>
{
   public:
    struct TrajectoryPart
    {
        double end_time_sec = 0;
        double position     = 0;
        double velocity     = 0;
        double acceleration = 0;
        double jerk         = 0;
    };

    JerkLimitedTrajectory1D();

    void generate(double initial_pos, double final_pos, double initial_vel,
                  double initial_accel, double max_vel, double max_accel,
                  double max_decel, double max_jerk);

    double getPosition(double t_sec) const override;
    double getVelocity(double t_sec) const override;
    double getAcceleration(double t_sec) const override;
    double getTotalTime() const override;

    std::pair<double, double> getMinMaxPositions() const;
    const TrajectoryPart& getTrajectoryPart(size_t index) const;
    size_t getNumTrajectoryParts() const;

   private:
    struct JerkPhase
    {
        double jerk;
        double duration;
    };
    struct AccelPlan
    {
        double total_distance = 0;
        std::vector<JerkPhase> phases;
    };

    static AccelPlan planAccelProfile(double initial_vel, double initial_accel,
                                      double final_vel, double max_jerk,
                                      double max_accel);

    static AccelPlan planDecelToStop(double initial_vel, double initial_accel,
                                     double max_jerk, double max_decel);

    static double closestPositionToStop(double initial_pos, double initial_vel,
                                        double initial_accel, double max_jerk,
                                        double max_decel);

    void generateDirect(double start_pos, double final_pos, double initial_vel,
                        double initial_accel, double max_vel, double max_accel,
                        double max_decel, double max_jerk, double time_offset);

    static void integrateState(double& p, double& v, double& a, double dt, double jerk);

    void applyPlan(const AccelPlan& plan, double start_pos, double& p, double& v,
                   double& a, double& t);

    size_t getTrajectoryIndexAtTime(double t_sec) const;
    std::pair<TrajectoryPart, double> getTrajPartAndDeltaTime(double t_sec) const;
    void addTrajectoryPart(const TrajectoryPart& part);

    std::vector<TrajectoryPart> trajectory_parts;

    static constexpr double EPSILON = 1e-9;
};
