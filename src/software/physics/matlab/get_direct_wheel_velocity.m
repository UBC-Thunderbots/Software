function wheel_velocity = get_direct_wheel_velocity(euclidean_velocity)
% GET_DIRECT_WHEEL_VELOCITY Directly converts euclidean velocity to wheel velocity.

% Robot Parameters
delta_t = 1/200; % s
robot_mass_M = 2.5; % kg
robot_radius_R = 0.09; % m
inertial_factor_alpha = 0.37; % m
front_wheel_angle_phi = deg2rad(32.06); % rad
rear_wheel_angle_theta = deg2rad(46.04); % rad
robot_max_acceleration = 1.0; % m/s^2

D = [ -sin(front_wheel_angle_phi) cos(front_wheel_angle_phi) 1;
    -sin(front_wheel_angle_phi) -cos(front_wheel_angle_phi) 1;
    sin(rear_wheel_angle_theta) -cos(rear_wheel_angle_theta) 1;
    sin(rear_wheel_angle_theta) cos(rear_wheel_angle_theta) 1];

wheel_velocity = D * euclidean_velocity;

end
