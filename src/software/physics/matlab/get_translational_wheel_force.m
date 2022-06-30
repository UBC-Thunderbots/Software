function translational_wheel_force = get_translational_wheel_force(euclidean_acceleration)
%GET_TRANSLATIONAL_WHEEL_FORCE Calculates the translational wheel force from a euclidean_acceleration.

% Robot Parameters
delta_t = 1/200; % s
robot_mass_M = 2.5; % kg
robot_radius_R = 0.09; % m
inertial_factor_alpha = 0.37; % m
front_wheel_angle_phi = deg2rad(32.06); % rad
rear_wheel_angle_theta = deg2rad(46.04); % rad
robot_max_acceleration = 1.0; % m/s^2

% Calculate translational wheel force
ax = [euclidean_acceleration(1) / sin(front_wheel_angle_phi); euclidean_acceleration(1) / sin(rear_wheel_angle_theta)];
ax_select = [-1 0; -1 0; 0 1; 0 1];

ay = [euclidean_acceleration(2) / cos(front_wheel_angle_phi); euclidean_acceleration(2) / cos(rear_wheel_angle_theta)];
ay_select = [1 0; -1 0; 0 -1; 0 1];

translational_wheel_force = robot_mass_M * (ax_select * ax + ay_select * ay);

end
