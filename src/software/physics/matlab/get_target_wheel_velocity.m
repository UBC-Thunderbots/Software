function target_wheel_velocity = get_target_wheel_velocity(current_wheel_velocity, target_euclidean_velocity)
%EUCLIDEAN_TO_WHEEL Converts Euclidean velocity to robot wheel velocity.
%   Step 1: Convert Euclidean velocity to acceleration. 
%   Step 2: Calculate translational wheel force. 
%   Step 3: Calculate rotational wheel force.
%   Step 4: Sum the wheel force.
%   Step 5: Convert wheel force to velocity.

% Robot Parameters
delta_t = 1/200; % s
robot_mass_M = 2.5; % kg
robot_radius_R = 0.09; % m
inertial_factor_alpha = 0.37; % m
front_wheel_angle_phi = deg2rad(32.06); % rad
rear_wheel_angle_theta = deg2rad(46.04); % rad
robot_max_acceleration = 1.0; % m/s^2

% Step 1: Calculate Euclidean acceleration
current_euclidean_velocity = get_euclidean_velocity(current_wheel_velocity);
calculated_euclidean_acceleration = (target_euclidean_velocity - current_euclidean_velocity) / delta_t;
max_euclidean_acceleration = abs(max(calculated_euclidean_acceleration));

% if acceleration exceeds max_robot_acceleration, scale acceleration so the max acceleration is max_robot_acceleration
if (robot_max_acceleration < max_euclidean_acceleration)
    target_euclidean_acceleration = robot_max_acceleration * (calculated_euclidean_acceleration / max_euclidean_acceleration);
else
    target_euclidean_acceleration = calculated_euclidean_acceleration;
end

% Step 2: Calculate translational wheel force
translational_wheel_force = get_translational_wheel_force(target_euclidean_acceleration);

% Step 3: Calculate rotational wheel force
rotational_wheel_force = get_rotational_wheel_force(target_euclidean_acceleration);

% Step 4: Sum the wheel forces
target_wheel_force = translational_wheel_force + rotational_wheel_force;

% Step 5: Convert wheel force to velocity
target_wheel_velocity = current_wheel_velocity + get_wheel_velocity(target_wheel_force);

end
