function wheel_velocity = get_wheel_velocity(wheel_force)

% Robot Parameters
delta_t = 1/200; % s
robot_mass_M = 2.5; % kg
robot_radius_R = 0.09; % m
inertial_factor_alpha = 0.37; % m
front_wheel_angle_phi = deg2rad(32.06); % rad
rear_wheel_angle_theta = deg2rad(46.04); % rad
robot_max_acceleration = 1.0; % m/s^2

% Wheel force to acceleration coupling matrix
a = sin(front_wheel_angle_phi)^2 - cos(front_wheel_angle_phi)^2;
b = -sin(front_wheel_angle_phi) * sin(rear_wheel_angle_theta) - cos(front_wheel_angle_phi) * cos(rear_wheel_angle_theta);
c = -sin(front_wheel_angle_phi) * sin(rear_wheel_angle_theta) + cos(front_wheel_angle_phi) * cos(rear_wheel_angle_theta);
d = sin(rear_wheel_angle_theta)^2 - cos(rear_wheel_angle_theta)^2;

DC_alpha = 1 / (robot_mass_M*inertial_factor_alpha) * ones(4) + 1 / robot_mass_M * [1 a b c; a 1 c b; b c 1 d; c b d 1];

% Convert wheel force to velocity
wheel_velocity = delta_t * DC_alpha * wheel_force;

end
