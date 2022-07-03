function euclidean_velocity = get_euclidean_velocity(wheel_velocity)
%GET_EUCLIDEAN_VELOCITY Calculates the euclidean velocity from the wheel velocity.

% Robot Parameters
delta_t = 1/200; % s
robot_mass_M = 2.5; % kg
robot_radius_R = 0.09; % m
inertial_factor_alpha = 0.37; % m
front_wheel_angle_phi = deg2rad(32.06); % rad
rear_wheel_angle_theta = deg2rad(46.04); % rad
robot_max_acceleration = 1.0; % m/s^2

% Euclidean velocity to wheel velocity coupling matrix
i = 1 / (2 * sin(front_wheel_angle_phi) + 2 * sin(rear_wheel_angle_theta));
j = cos(front_wheel_angle_phi) / (2 * cos(front_wheel_angle_phi)^2 + 2 * cos(rear_wheel_angle_theta)^2);
k = sin(rear_wheel_angle_theta) / (2 * sin(front_wheel_angle_phi) + 2 * sin(rear_wheel_angle_theta));

D_inverse = [-i -i i i; j -j -(1 - j) (1 - j); k k (1 - k) (1 - k)];

% Calculate Euclidean velocity from wheel velocity
euclidean_velocity = D_inverse * wheel_velocity;
