function rotational_wheel_force = get_rotational_wheel_force(euclidean_acceleration)
%GET_ROTATIONAL_WHEEL_FORCE Calculates the rotational wheel force from the euclidean acceleration.

% Robot Parameters
delta_t = 1/200; % s
robot_mass_M = 2.5; % kg
robot_radius_R = 0.09; % m
inertial_factor_alpha = 0.37; % m
front_wheel_angle_phi = deg2rad(32.06); % rad
rear_wheel_angle_theta = deg2rad(46.04); % rad
robot_max_acceleration = 1.0; % m/s^2

% Calculate rotational wheel force
rotational_wheel_force = euclidean_acceleration(3) / 4 * inertial_factor_alpha * robot_mass_M * robot_radius_R;

end
