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

RefboxTeamInfo us
RefboxTeamInfo them

Point2D designated_position

# if our team is on the positive X axis (east) side
bool our_team_on_east_side
