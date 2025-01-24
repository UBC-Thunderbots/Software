import math
import redis
from software.py_constants import *
from proto.import_all_protos import *
from software.embedded.constants.py_constants import (ROBOT_MAX_ANG_SPEED_RAD_PER_S, ROBOT_MAX_SPEED_M_PER_S,
                                                      MAX_FORCE_DRIBBLER_SPEED_RPM)


class EmbeddedData:
    """Model class responsible for interfacing with onboard disk data on the robot"""
    def __init__(self) -> None:
        # Initializes the redis cache connection
        self.redis = redis.StrictRedis(
            host=REDIS_DEFAULT_HOST,
            port=REDIS_DEFAULT_PORT,
            charset="utf-8",
            decode_responses=True
        )

    def get_robot_id(self) -> int:
        return int(self.redis.get(ROBOT_ID_REDIS_KEY))

    def get_network_interface(self) -> str:
        return str(self.redis.get(ROBOT_NETWORK_INTERFACE_REDIS_KEY))

    def get_channel_id(self) -> str:
        return str(self.redis.get(ROBOT_MULTICAST_CHANNEL_REDIS_KEY))

    def get_kick_constant(self) -> str:
        return str(self.redis.get(ROBOT_KICK_CONSTANT_REDIS_KEY))

    def get_kick_coeff(self) -> str:
        return str(self.redis.get(ROBOT_KICK_EXP_COEFF_REDIS_KEY))

    def get_chip_pulse_width(self) -> str:
        return str(self.redis.get(ROBOT_CHIP_PULSE_WIDTH_REDIS_KEY))

    def get_current_draw(self) -> str:
        return str(self.redis.get(ROBOT_CURRENT_DRAW_REDIS_KEY))

    def get_battery_volt(self) -> str:
        return str(self.redis.get(ROBOT_BATTERY_VOLTAGE_REDIS_KEY))

    def get_cap_volt(self) -> str:
        return str(self.redis.get(ROBOT_CAPACITOR_VOLTAGE_REDIS_KEY))

    def __clamp(self, val: float, min_val: float, max_val: float) -> float:
        """Simple Math Clamp function (Faster than numpy & fewer dependencies)
        :param val: Value to clamp
        :param min_val: Minimum (Lower) Bound
        :param max_val: Maximum (Upper) Bound
        """
        return min(max(val, min_val), max_val)

    def get_rotate_primitive(self, velocity: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a velocity
        :param velocity: Angular Velocity to rotate the robot
        """
        velocity = self.__clamp(
            velocity,
            -ROBOT_MAX_ANG_SPEED_RAD_PER_S,
            ROBOT_MAX_ANG_SPEED_RAD_PER_S
        )
        motor_control_primitive = MotorControl()
        motor_control_primitive.direct_velocity_control.angular_velocity.radians_per_second = velocity
        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive,
            power_control=PowerControl()
        )
        return Primitive(direct_control=direct_control_primitive)

    def get_move_primitive(self, angle: float, speed: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a speed.
        :param angle: Angle to move the robot at in degrees
        :param speed: Speed to move the robot at in m/s
        """
        speed = self.__clamp(
            val=speed,
            min_val=0,
            max_val=ROBOT_MAX_SPEED_M_PER_S
        )
        motor_control_primitive = MotorControl()
        motor_control_primitive.direct_velocity_control.velocity.x_component_meters = speed * math.cos(angle)
        motor_control_primitive.direct_velocity_control.velocity.y_component_meters = speed * math.sin(angle)
        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive,
            power_control=PowerControl()
        )
        return Primitive(direct_control=direct_control_primitive)

    def get_chip_primitive(self, auto: bool, distance: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a distance and state.
        :param auto: Determines whether auto-chip is enabled
        :param distance: Distance to chip the "ball"
        """
        distance = self.__clamp(
            val=distance,
            min_val=0,
            max_val=ROBOT_MAX_SPEED_M_PER_S
        )
        power_control_primitive = PowerControl()
        if not auto:
            power_control_primitive.chicker.chip_distance_meters = distance
        else:
            # TODO: Change this to a constant from somewhere else
            power_control_primitive.chicker.auto_chip_or_kick.autochip_distance_meters = 1.5
        direct_control_primitive = DirectControlPrimitive(
            motor_control=MotorControl(),
            power_control=power_control_primitive
        )
        return Primitive(direct_control=direct_control_primitive)

    def get_kick_primitive(self, auto: bool, speed: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a speed and state.
        :param auto: Determines whether auto-kick is enabled
        :param speed: Speed to kick the "ball" at
        """
        speed = self.__clamp(
            val=speed,
            min_val=0,
            max_val=ROBOT_MAX_SPEED_M_PER_S
        )
        power_control_primitive = PowerControl()
        if not auto:
            power_control_primitive.chicker.kick_speed_m_per_s = speed
        else:
            # TODO: Change this to a constant from somewhere else
            power_control_primitive.chicker.auto_chip_or_kick.autokick_speed_m_per_s = 1.5
        direct_control_primitive = DirectControlPrimitive(
            motor_control=MotorControl(),
            power_control=power_control_primitive
        )
        return Primitive(direct_control=direct_control_primitive)

    def get_dribble_primitive(self, velocity: float) -> Primitive:
        """Prepares and returns the processed direct control primitive given a velocity.
        :param velocity: Speed & direction of the dribbler
        """
        velocity = self.__clamp(
            val=velocity,
            min_val=-MAX_FORCE_DRIBBLER_SPEED_RPM,
            max_val=MAX_FORCE_DRIBBLER_SPEED_RPM
        )
        motor_control_primitive = MotorControl()
        motor_control_primitive.dribbler_speed_rpm = int(velocity)
        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive,
            power_control=PowerControl()
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
            min_val=-ROBOT_MAX_SPEED_M_PER_S,
            max_val=ROBOT_MAX_SPEED_M_PER_S
        )

        for wheel in wheels:
            wheel_velocity_map[wheel] = velocity
        motor_control_primitive = MotorControl()
        motor_control_primitive.direct_per_wheel_control.front_left_wheel_velocity = wheel_velocity_map[1]
        motor_control_primitive.direct_per_wheel_control.back_left_wheel_velocity = wheel_velocity_map[2]
        motor_control_primitive.direct_per_wheel_control.front_right_wheel_velocity = wheel_velocity_map[3]
        motor_control_primitive.direct_per_wheel_control.back_right_wheel_velocity = wheel_velocity_map[4]

        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive,
            power_control=PowerControl()
        )
        return Primitive(direct_control=direct_control_primitive)
