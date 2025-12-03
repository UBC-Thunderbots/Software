import math
import os
import tomllib
from software.py_constants import *
from proto.import_all_protos import *
from software.embedded.constants.py_constants import (
    ROBOT_MAX_ANG_SPEED_RAD_PER_S,
    ROBOT_MAX_SPEED_M_PER_S,
    MAX_FORCE_DRIBBLER_SPEED_RPM,
)
from software.py_constants import (
    AUTO_CHIP_DISTANCE_DEFAULT_M,
    AUTO_KICK_SPEED_DEFAULT_M_PER_S,
    WHEEL_ROTATION_MAX_SPEED_M_PER_S,
)


class EmbeddedData:
    """Model class responsible for interfacing with onboard disk data on the robot.
    This class manages static data on the robot as well as the operations necessary with mutating data for use
    """

    def __init__(self) -> None:
        self.config_file_path = "/opt/tbotspython/robot_config.toml"
        self.config = self._load_config()
        self.epoch_timestamp_seconds = 0
        self.battery_voltage = 0
        self.primitive_packet_loss_percentage = 0
        self.primitive_executor_step_time_ms = 0

    def _load_config(self) -> dict:
        """Load TOML configuration file."""
        if not os.path.exists(self.config_file_path):
            return {}
        try:
            with open(self.config_file_path, "rb") as f:
                return tomllib.load(f)
        except Exception as e:
            print(f"Warning: Failed to load TOML config from {self.config_file_path}: {e}")
            return {}

    def _get_value(self, key: str):
        """Get a value from the TOML file; TOML keys do not start with '/'."""
        normalized_key = key.lstrip("/")
        return self.config.get(normalized_key, "")

    def get_robot_id(self) -> str:
        return self._get_value(ROBOT_ID_CONFIG_KEY)

    def get_network_interface(self) -> str:
        return self._get_value(ROBOT_NETWORK_INTERFACE_CONFIG_KEY)

    def get_channel_id(self) -> str:
        return self._get_value(ROBOT_MULTICAST_CHANNEL_CONFIG_KEY)

    def get_kick_constant(self) -> str:
        return self._get_value(ROBOT_KICK_CONSTANT_CONFIG_KEY)

    def get_kick_coeff(self) -> str:
        return self._get_value(ROBOT_KICK_EXP_COEFF_CONFIG_KEY)

    def get_chip_pulse_width(self) -> str:
        return self._get_value(ROBOT_CHIP_PULSE_WIDTH_CONFIG_KEY)

    def get_current_draw(self) -> str:
        return self._get_value(ROBOT_CURRENT_DRAW_CONFIG_KEY)

    def get_battery_volt(self) -> str:
        return self._get_value(ROBOT_BATTERY_VOLTAGE_CONFIG_KEY)

    def get_cap_volt(self) -> str:
        return self._get_value(ROBOT_CAPACITOR_VOLTAGE_CONFIG_KEY)

    def __clamp(self, val: float, min_val: float, max_val: float) -> float:
        """Simple Math Clamp function (Faster than numpy & fewer dependencies)
        :param val: Value to clamp
        :param min_val: Minimum (Lower) Bound
        :param max_val: Maximum (Upper) Bound
        """
        return min(max(val, min_val), max_val)

    # TODO (#3435): Refactor Get Primitives
    def get_rotate_primitive(self, velocity: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a velocity
        :param velocity: Angular Velocity to rotate the robot
        """
        velocity = self.__clamp(
            velocity, -ROBOT_MAX_ANG_SPEED_RAD_PER_S, ROBOT_MAX_ANG_SPEED_RAD_PER_S
        )
        motor_control_primitive = MotorControl()
        motor_control_primitive.direct_velocity_control.angular_velocity.radians_per_second = velocity
        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive, power_control=PowerControl()
        )
        return Primitive(direct_control=direct_control_primitive)

    def get_move_primitive(self, angle: float, speed: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a speed.
        :param angle: Angle to move the robot at in degrees
        :param speed: Speed to move the robot at in m/s
        """
        speed = self.__clamp(val=speed, min_val=0, max_val=ROBOT_MAX_SPEED_M_PER_S)
        motor_control_primitive = MotorControl()
        motor_control_primitive.direct_velocity_control.velocity.x_component_meters = (
            speed * math.cos(angle)
        )
        motor_control_primitive.direct_velocity_control.velocity.y_component_meters = (
            speed * math.sin(angle)
        )
        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive, power_control=PowerControl()
        )
        return Primitive(direct_control=direct_control_primitive)

    def get_chip_primitive(self, auto: bool, distance: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a distance and state.
        :param auto: Determines whether auto-chip is enabled
        :param distance: Distance to chip the "ball"
        """
        distance = self.__clamp(
            val=distance, min_val=0, max_val=ROBOT_MAX_SPEED_M_PER_S
        )
        power_control_primitive = PowerControl()
        if not auto:
            power_control_primitive.chicker.chip_distance_meters = distance
        else:
            power_control_primitive.chicker.auto_chip_or_kick.autochip_distance_meters = AUTO_CHIP_DISTANCE_DEFAULT_M
        direct_control_primitive = DirectControlPrimitive(
            motor_control=MotorControl(), power_control=power_control_primitive
        )
        return Primitive(direct_control=direct_control_primitive)

    def get_zero_power_control_primitive(self) -> Primitive:
        """Creates a PowerControl primitive with zeroed/default base values"""
        power_control_primitive = PowerControl()
        power_control_primitive.geneva_slot = Slot.CENTRE_RIGHT
        return power_control_primitive

    def get_zero_motor_control_primitive(self) -> Primitive:
        """Creates a MotorControl primitive with zeroed/default base values"""
        motor_control_primitive = MotorControl()
        motor_control_primitive.direct_velocity_control.velocity.x_component_meters = 0
        motor_control_primitive.direct_velocity_control.velocity.y_component_meters = 0
        motor_control_primitive.direct_velocity_control.angular_velocity.radians_per_second = 0
        return motor_control_primitive

    def get_kick_primitive(self, auto: bool, speed: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a speed and state.
        :param auto: Determines whether auto-kick is enabled
        :param speed: Speed to kick the "ball" at
        """
        speed = self.__clamp(val=speed, min_val=0, max_val=ROBOT_MAX_SPEED_M_PER_S)
        power_control_primitive = self.get_zero_power_control_primitive()
        if not auto:
            power_control_primitive.chicker.kick_speed_m_per_s = speed
        else:
            power_control_primitive.chicker.auto_chip_or_kick.autokick_speed_m_per_s = (
                AUTO_KICK_SPEED_DEFAULT_M_PER_S
            )
        direct_control_primitive = DirectControlPrimitive(
            motor_control=self.get_zero_motor_control_primitive(),
            power_control=power_control_primitive,
        )
        return Primitive(direct_control=direct_control_primitive)

    def get_dribble_primitive(self, velocity: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a velocity.
        :param velocity: Speed & direction of the dribbler
        """
        velocity = self.__clamp(
            val=velocity,
            min_val=MAX_FORCE_DRIBBLER_SPEED_RPM,
            max_val=-MAX_FORCE_DRIBBLER_SPEED_RPM,
        )
        motor_control_primitive = self.get_zero_motor_control_primitive()
        motor_control_primitive.dribbler_speed_rpm = int(velocity)
        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive, power_control=PowerControl()
        )
        return Primitive(direct_control=direct_control_primitive)

    def get_move_wheel_primitive(self, wheels: list[int], velocity: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a velocity mapped to
        wheel_velocity_map = {1: 0, 2: 0, 3: 0, 4: 0} where {1:"NE", 2:"SE", 3:"SW", 4:"NW"}
        :param wheels: The wheels to rotate
        :param velocity: The speed & direction to rotate at
        """
        wheel_velocity_map = {1: 0, 2: 0, 3: 0, 4: 0}
        velocity = self.__clamp(
            val=velocity,
            min_val=-WHEEL_ROTATION_MAX_SPEED_M_PER_S,
            max_val=WHEEL_ROTATION_MAX_SPEED_M_PER_S,
        )

        for wheel in wheels:
            wheel_velocity_map[wheel] = velocity
        motor_control_primitive = MotorControl()
        motor_control_primitive.direct_per_wheel_control.front_left_wheel_velocity = (
            wheel_velocity_map[1]
        )
        motor_control_primitive.direct_per_wheel_control.back_left_wheel_velocity = (
            wheel_velocity_map[2]
        )
        motor_control_primitive.direct_per_wheel_control.front_right_wheel_velocity = (
            wheel_velocity_map[3]
        )
        motor_control_primitive.direct_per_wheel_control.back_right_wheel_velocity = (
            wheel_velocity_map[4]
        )

        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive, power_control=PowerControl()
        )
        return Primitive(direct_control=direct_control_primitive)
