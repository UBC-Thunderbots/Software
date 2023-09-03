#pragma once

class KinematicConstraints
{
   public:
    KinematicConstraints() = delete;

    KinematicConstraints(double max_velocity, double max_acceleration,
                         double max_deceleration);

    double getMaxVelocity() const;
    double getMaxAcceleration() const;
    double getMaxDeceleration() const;

    void setMaxVelocity(double max_velocity);
    void setMaxAcceleration(double max_acceleration);
    void setMaxDeceleration(double max_deceleration);


   private:
    double max_velocity;
    double max_acceleration;
    double max_deceleration;
};
