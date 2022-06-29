function target_wheel_speeds = euclidean_to_wheel(current_wheel_speeds, target_euclidean_velocity)
%EUCLIDEAN_TO_WHEEL Converts Euclidean velocity to robot wheel speeds.
%   Step 1: Convert Euclidean velocity to acceleration. 
%   Step 2: Calculate translational wheel forces. 
%   Step 3: Calculate rotational wheel forces.
%   Step 4: Sum the wheel forces.
%   Step 5: Convert wheel forces to speeds.

% Robot Parameters
delta_t = 1/200; % s
robot_mass_M = 2.5; % kg
robot_radius_R = 0.09; % m
inertial_factor_alpha = 0.37; % m
front_wheel_angle_phi = deg2rad(50.0); % rad
rear_wheel_angle_theta = deg2rad(46.04); % rad

% Wheel force to acceleration coupling matrix
a = sin(front_wheel_angle_phi)^2 - cos(front_wheel_angle_phi)^2;
b = -sin(front_wheel_angle_phi) * sin(rear_wheel_angle_theta) - cos(front_wheel_angle_phi) * cos(rear_wheel_angle_theta);
c = -sin(front_wheel_angle_phi) * sin(rear_wheel_angle_theta) + cos(front_wheel_angle_phi) * cos(rear_wheel_angle_theta);
d = sin(rear_wheel_angle_theta)^2 - cos(rear_wheel_angle_theta)^2;

DC_alpha = 1 / (robot_mass_M*inertial_factor_alpha) * ones(4) + 1 / robot_mass_M * [1 a b c; a 1 c b; b c 1 d; c b d 1];

% Euclidean velocity to wheel speed coupling matrix
i = 1 / (2 * sin(front_wheel_angle_phi) + 2 * sin(rear_wheel_angle_theta));
j = cos(front_wheel_angle_phi) / (2 * cos(front_wheel_angle_phi)^2 + 2 * cos(rear_wheel_angle_theta)^2);
k = sin(rear_wheel_angle_theta) / (2 * sin(front_wheel_angle_phi) + 2 * sin(rear_wheel_angle_theta));

D_inverse = [-i -i i i; j -j -(1 - j) (1 - j); k k (1 - k) (1 - k)];

% Step 1: Calculate Euclidean acceleration
current_euclidean_velocity = D_inverse * current_wheel_speeds;
target_euclidean_acceleration = (target_euclidean_velocity - current_euclidean_velocity) / delta_t;

% Step 2: Calculate translational wheel force
ax = [target_euclidean_acceleration(1) / sin(front_wheel_angle_phi); target_euclidean_acceleration(1) / sin(rear_wheel_angle_theta)];
ax_select = [-1 0; -1 0; 0 1; 0 1];

ay = [target_euclidean_acceleration(2) / cos(front_wheel_angle_phi); target_euclidean_acceleration(2) / cos(rear_wheel_angle_theta)];
ay_select = [1 0; -1 0; 0 -1; 0 1];

translational_wheel_force = robot_mass_M * (ax_select * ax + ay_select * ay);

% Step 3: Calculate rotational wheel force
rotational_wheel_force = target_euclidean_acceleration(3) / 4 * inertial_factor_alpha * robot_mass_M * robot_radius_R;

% Step 4: Sum the wheel forces
target_wheel_force = translational_wheel_force + rotational_wheel_force;

% Step 5: Conver wheel forces to speeds
target_delta_wheel_speeds = delta_t * DC_alpha * target_wheel_force;
target_wheel_speeds = current_wheel_speeds + target_delta_wheel_speeds;

end

