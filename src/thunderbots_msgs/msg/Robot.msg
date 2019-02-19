# Describes the current state of a robot

# The timestamp of this data in seconds
float64 timestamp_seconds

# The id of the robot
uint32 id

# The robot's position, with coordinates in metres
Point2D position

# The robot's velocity, a vector in m/s
Point2D velocity

# The orientation of the robot, from [-PI, PI] in Radians
float64 orientation

# The angular velocity of the robot, in Radians/second
float64 angular_velocity
