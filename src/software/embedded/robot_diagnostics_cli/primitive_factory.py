import math
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


class PrimitiveFactory:
    def __init__(self) -> None:
        pass

    @staticmethod
    def get_primitive(self, **kwargs) -> Primitive:
        if len(kwargs) != 1:
            raise ValueError("Primitive constructor takes exactly 1 argument")

        primitive_type, args = next(iter(kwargs.items()))

        if not isinstance(args, (list, tuple)):
            raise TypeError(
                f"Arguments for '{primitive_type}' must be a list or tuple"
            )

        motor_control_primitive = MotorControl()
        power_control_primitive = PowerControl()

        match primitive_type:
            case "rotate": # rotate=[velocity]
                self._rotate(motor_control_primitive, *args)
            case "move": # move=[angle, speed]
                self._move(motor_control_primitive, *args)
            case "chip": # chip=[auto, distance]
                self._chip(power_control_primitive, *args)
            case "kick": # kick=[auto, speed]
                self._kick(motor_control_primitive, power_control_primitive, *args)
            case "dribble": # dribble=[velocity]
                self._dribble(motor_control_primitive, *args)
            case "move_wheel": # move_wheels=[wheels, velocity]
                self._move_wheel(motor_control_primitive, *args)
            case "zero_power": # zero_power=[]
                self._zero_power(power_control_primitive)
                return power_control_primitive
            case "zero_motor": # zero_motor=[]
                self._zero_motor(motor_control_primitive)
                return motor_control_primitive
            case _:
                raise ValueError(f"Unknown primitive type: '{primitive_type}'")
        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive, power_control=power_control_primitive
        )
        return Primitive(direct_control=direct_control_primitive)

    def _rotate(self, motor_control_primitive, velocity: float) -> None:
        velocity = self._clamp(
            velocity, -ROBOT_MAX_ANG_SPEED_RAD_PER_S, ROBOT_MAX_ANG_SPEED_RAD_PER_S
        )
        motor_control_primitive.direct_velocity_control.angular_velocity.radians_per_second = velocity

    def _move(self, motor_control_primitive, angle: float, speed: float) -> None:
        speed = self._clamp(val=speed, min_val=0, max_val=ROBOT_MAX_SPEED_M_PER_S)
        motor_control_primitive.direct_velocity_control.velocity.x_component_meters = (
            speed * math.cos(angle)
        )
        motor_control_primitive.direct_velocity_control.velocity.y_component_meters = (
            speed * math.sin(angle)
        )

    def _chip(self, power_control_primitive, auto: bool, distance: float) -> None:
        distance = self._clamp(
            val=distance, min_val=0, max_val=ROBOT_MAX_SPEED_M_PER_S
        )
        if not auto:
            power_control_primitive.chicker.chip_distance_meters = distance
        else:
            power_control_primitive.chicker.auto_chip_or_kick.autochip_distance_meters = AUTO_CHIP_DISTANCE_DEFAULT_M

    def _kick(self, motor_control_primitive, power_control_primitive, auto: bool, speed: float) -> None:
        speed = self._clamp(val=speed, min_val=0, max_val=ROBOT_MAX_SPEED_M_PER_S)
        self._zero_power(power_control_primitive)
        self._zero_motor(motor_control_primitive)
        if not auto:
            power_control_primitive.chicker.kick_speed_m_per_s = speed
        else:
            power_control_primitive.chicker.auto_chip_or_kick.autokick_speed_m_per_s = (
                AUTO_KICK_SPEED_DEFAULT_M_PER_S
            )

    def _dribble(self, motor_control_primitive, velocity: float):
        velocity = self._clamp(
            val=velocity,
            min_val=MAX_FORCE_DRIBBLER_SPEED_RPM,
            max_val=-MAX_FORCE_DRIBBLER_SPEED_RPM,
        )
        self._zero_motor(motor_control_primitive)
        motor_control_primitive.dribbler_speed_rpm = int(velocity)

    def _move_wheel(self, motor_control_primitive, wheels: list[int], velocity: float) -> None:
        wheel_velocity_map = {1: 0, 2: 0, 3: 0, 4: 0}
        velocity = self._clamp(
            val=velocity,
            min_val=-WHEEL_ROTATION_MAX_SPEED_M_PER_S,
            max_val=WHEEL_ROTATION_MAX_SPEED_M_PER_S,
        )
        for wheel in wheels:
            wheel_velocity_map[wheel] = velocity
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

    def _zero_power(self, power_control_primitive) -> None:
        power_control_primitive.geneva_slot = Slot.CENTRE_RIGHT

    def _zero_motor(self, motor_control_primitive) -> None:
        motor_control_primitive.direct_velocity_control.velocity.x_component_meters = 0
        motor_control_primitive.direct_velocity_control.velocity.y_component_meters = 0
        motor_control_primitive.direct_velocity_control.angular_velocity.radians_per_second = 0

    def _clamp(self, val: float, min_val: float, max_val: float) -> float:
        return min(max(val, min_val), max_val)
