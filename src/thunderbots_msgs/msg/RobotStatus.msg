# This message contains various robot diagnostics (e.g. voltages, link quality, etc.)
# and messages related to robot and dongle condition.

# The robot number.
int32 robot


# Messages related to the robot that are to be displayed in the visualizer
# (usually errors that need attention)
string[] robot_messages


# Messages that are related to the dongle itself
# (usually errors that need attention)
string[] dongle_messages


# The maximum possible kick speed, in m/s.
float64 kick_speed_max


# The kick resolution for HScale.
float64 kick_speed_resolution


# The maximum possible chip distance, in m.
float64 chip_distance_max


# The chip resolution for HScale.
float64 chip_distance_resolution


# The maximum power level understood by the direct_dribbler function.
uint32 direct_dribbler_max


# Whether or not the robot is currently responding to radio communication.
bool alive


# Whether the robot is in direct mode.
bool direct_control


# Whether or not the ball is interrupting the robot’s laser beam.
bool ball_in_beam


# Whether or not the robot’s capacitor is charged enough to kick the ball.
bool capacitor_charged

# Indicates when autokick has been fired.
bool autokick_fired


# The voltage on the robot’s battery, in volts.
float64 battery_voltage


# The voltage on the robot’s kicking capacitor, in volts.
float64 capacitor_voltage


# The reading of the robot’s laser sensor.
float64 break_beam_reading


# The rough maximum full-scale deflection (maximum possible reading) of the laser sensor.
float64 break_beam_scale


# The temperature of the robot’s dribbler motor, in degrees Celsius.
float64 dribbler_temperature


# The speed of the robot’s dribbler motor, in revolutions per minute.
int32 dribbler_speed


# The temperature of the robot’s mainboard, in degrees Celsius.
float64 board_temperature


# The link quality of the last received packet, from 0 (worst) to 1 (best).
float64 link_quality


# The received signal strength of the last received packet, in decibels.
int32 received_signal_strength_db


# Whether or not the build ID information is valid.
bool build_ids_valid


# The microcontroller firmware build ID.
uint32 fw_build_id


# The FPGA bitstream build ID.
uint32 fpga_build_id

