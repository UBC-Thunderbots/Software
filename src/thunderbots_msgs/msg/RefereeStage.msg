
# The stage of the game.
# This can take on any of the values defined below.
int32 stage


# The first half is about to start.
# A kickoff is called within this stage.
# This stage ends with the NORMAL_START.
int32 NORMAL_FIRST_HALF_PRE = 0
# The first half of the normal game, before half time.
int32 NORMAL_FIRST_HALF = 1
# Half time between first and second halves.
int32 NORMAL_HALF_TIME = 2
# The second half is about to start.
# A kickoff is called within this stage.
# This stage ends with the NORMAL_START.
int32 NORMAL_SECOND_HALF_PRE = 3
# The second half of the normal game, after half time.
int32 NORMAL_SECOND_HALF = 4
# The break before extra time.
int32 EXTRA_TIME_BREAK = 5
# The first half of extra time is about to start.
# A kickoff is called within this stage.
# This stage ends with the NORMAL_START.
int32 EXTRA_FIRST_HALF_PRE = 6
# The first half of extra time.
int32 EXTRA_FIRST_HALF = 7
# Half time between first and second extra halves.
int32 EXTRA_HALF_TIME = 8
# The second half of extra time is about to start.
# A kickoff is called within this stage.
# This stage ends with the NORMAL_START.
int32 EXTRA_SECOND_HALF_PRE = 9
# The second half of extra time.
int32 EXTRA_SECOND_HALF = 10
# The break before penalty shootout.
int32 PENALTY_SHOOTOUT_BREAK = 11
# The penalty shootout.
int32 PENALTY_SHOOTOUT = 12
# The game is over.
int32 POST_GAME = 13
