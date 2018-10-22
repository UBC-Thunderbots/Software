
string name

# The amount of goals scored during normal play and overtime.
uint32 score

uint32 red_cards

# The amount of time (in microseconds) left on each yellow card issued to the team.
# If no yellow cards are issued, this array has no elements.
# Otherwise, times are ordered from smallest to largest.
uint32[] yellow_card_times

# The total number of yellow cards issued during this game.
uint32 yellow_cards

# The number of timeouts this team can still call.
# If in a timeout right now, that timeout is excluded.
uint32 timeouts

# The number of microseconds of timeout this team can use.
uint32 timeout_time

# The pattern number of this team's goalie.
uint32 goalie
