# This message generically defines a low-level Primitive

# The name of the primitive. We use a string because it is
# easier to read and debug than an integer.
string primitive_name

# The id of the robot the Primitive corresponds to
uint32 robot_id

# All of our Primitive parameters can be represented and passed
# as a double. For example, a destination x-coordinate, orientation.
# or kicking speed.
float64[] parameters

# Extra bits are used as flags for various behaviours or extra attribues.
# Examples are indicating if the robot should kick vs chip, or if autokick should
# be on.
bool[] extra_bits
