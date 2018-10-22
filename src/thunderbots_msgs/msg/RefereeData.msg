
# The UNIX timestamp when the packet was sent, in microseconds.
# Divide by 1,000,000 to get a time_t.
uint64 packet_timestamp

# The "coarse" stage of the game.
RefereeStage stage

# The number of microseconds left in the stage.
# Becomes negative when a stage runs over its specified time.
int32 stage_time_left

# The "fine" states of play.
RefereeCommand command

# Number of commands issued since startup.
# The ssl_referee.proto comments: (mod 2^32).
# This probably means that it wraps back to 0 when it overflows.
uint32 command_counter

# UNIX timestamp of when the command was issued, in microseconds.
# Value changes only when a new command gets issued, not on each packet.
uint64 command_timestamp

RefereeTeamInfo us
RefereeTeamInfo them

Vector2f designated_position

# Information about the direction of play.
# True, if the blue team will have it's goal on the positive x-axis of the ssl-vision coordinate system
# Obviously, the yellow team will play on the opposide half
# For compatibility, this field is optional
bool blueTeamOnPositiveHalf

# The game event that caused the referee command
GameEvent gameEvent