# This message describes a team of robots

# The robots that make up the team
Robot[] robots

# The robot id of the goalie for this team. A value < 0 indicates that no goalie
# is set for the team
int32 goalie_id

# The number of milliseconds a robot must not have
# been updated for before it is removed from the team
float64 robot_expiry_buffer_milliseconds
