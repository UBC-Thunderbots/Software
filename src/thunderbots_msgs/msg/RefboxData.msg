# The RefboxData message encapsulates all the data we obtain from Refbox, including
# the stage of the game, information on each team (see RefboxTeamInfo.msg), the current
# Refbox command (e.g. free indirect kick for blue) and the ball placement point.

# Taken from https://github.com/RoboTeamTwente/roboteam_msgs/tree/master/msg/RefereeData.msg


# The UNIX timestamp when the packet was sent, in microseconds.
# Divide by 1,000,000 to get a time_t.
uint64 packet_timestamp

# The number of microseconds left in the stage.
# Becomes negative when a stage runs over its specified time.
int32 stage_time_left

# The "fine" states of play.
RefboxCommand command

# Number of commands issued since startup.
# wraps back to 0 when it overflows.
uint32 command_counter

# UNIX timestamp of when the command was issued, in microseconds.
# Value changes only when a new command gets issued, not on each packet.
uint64 command_timestamp

# Team info for us, including team name, score, and yellow/red card count and remaining time.
RefboxTeamInfo us
# Team info for other team
RefboxTeamInfo them

# The point for ball placement. See Robocup SSL Rules
Point2D ball_placement_point

# Information about the direction of play.
# True, if the blue team will have it's goal on the positive x-axis of the ssl-vision coordinate system
# Obviously, the yellow team will play on the opposite half
# For compatibility, this field is optional
bool blueTeamOnPositiveHalf
