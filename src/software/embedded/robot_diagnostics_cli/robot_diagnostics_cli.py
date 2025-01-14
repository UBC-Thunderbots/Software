import math

import typer as Typer
from rich import print
from rich.live import Live
from rich.table import Table
from rich.console import Console
from software.py_constants import *
from typer_shell import make_typer_shell
from software.embedded.constants.py_constants import (DEFAULT_PRIMITIVE_DURATION,
                                                      ROBOT_MAX_ANG_SPEED_RAD_PER_S, ROBOT_MAX_SPEED_M_PER_S,
                                                      MAX_FORCE_DRIBBLER_SPEED_RPM)
from proto.import_all_protos import *
import subprocess
import InquirerPy
from functools import wraps
from typing import List, Optional
from typing_extensions import Annotated
from embedded_communication import EmbeddedCommunication


class RobotDiagnosticsCLI:
    def __init__(self, embedded_communication: EmbeddedCommunication) -> None:
        """Setup constructor for the Shell CLI"""
        self.app = make_typer_shell(prompt="⚡ ")
        self.app.command(short_help="Toggles easy selection shell (For new users)")(self.toggle_simple_mode)
        self.app.command(short_help="Moves the robot")(self.move)
        self.app.command(short_help="Moves specific wheels of the robot")(self.move_wheel)
        self.app.command(short_help="Rotates the robot")(self.rotate)
        self.app.command(short_help="Spins the dribbler")(self.dribble)
        self.app.command(short_help="Chips the chipper")(self.chip)
        self.app.command(short_help="Kicks the kicker")(self.kick)
        self.app.command(short_help="Show Robot Status Info")(self.stats)
        self.app.command(short_help="Shows Redis Values")(self.redis)
        self.app.command(short_help="Prints Thunderloop Logs")(self.log)
        self.app.command(short_help="Prints Thunderloop Status")(self.status)
        self.app.command(short_help="Restarts Thunderloop")(self.restart_thunderloop)

        self.embedded_communication = embedded_communication
        self.embedded_data = self.embedded_communication.embedded_data

        self.console = Console()
        self.easy_mode_enabled = False



    def catch_interrupt_exception(exit_code=1):
        """Decorator for handling keyboard exceptions and safely clearing cached primitives"""

        def decorator(func):
            @wraps(func)
            def wrapper(self, *args, **kwargs):
                try:
                    self.embedded_communication.run_primitive_set(Primitive(stop=StopPrimitive()))
                    return func(self, *args, **kwargs)
                except KeyboardInterrupt:
                    print("E-Stop Activated: Stopped Primitive Send")
                    self.embedded_communication.run_primitive_set(Primitive(stop=StopPrimitive()))
                    raise Typer.Exit(code=exit_code)
                except Exception as e:
                    self.embedded_communication.run_primitive_set(Primitive(stop=StopPrimitive()))
                    print(f"Unknown Exception: {e}")
                    raise Typer.Exit(code=exit_code)
                finally:
                    self.embedded_communication.run_primitive_set(Primitive(stop=StopPrimitive()))

            return wrapper

        return decorator

    def __generate_stats_table(self) -> Table:
        """Make a new table with robot status information."""
        table = Table()
        table.add_column("Robot ID")
        table.add_column("Battery (V)")
        table.add_column("Thunderloop Status")
        table.add_column("Lifetime (s)")

        status = "[red]OFFLINE"
        if subprocess.call(["systemctl", "is-active", "--quiet", "thunderloop.service"]) == 0:
            status = "[green]ONLINE"

        table.add_row(
            f"{self.embedded_data.redis.get(ROBOT_ID_REDIS_KEY)}",
            f"{self.embedded_communication.battery_voltage}",
            status,
            f"{self.embedded_communication.epoch_timestamp_seconds}"
        )
        return table

    def __generate_redis_table(self) -> Table:
        """Make a new table with embedded_data.redis value information."""
        table = Table(show_header=True, header_style="bold blue")
        table.add_column("Redis Value Name")
        table.add_column("Key", style="dim")
        table.add_column("Value")

        table.add_row("Robot ID", f"{ROBOT_ID_REDIS_KEY}",
                      f"{self.embedded_data.redis.get(ROBOT_ID_REDIS_KEY)}")
        table.add_row("Channel ID", f"{ROBOT_MULTICAST_CHANNEL_REDIS_KEY}",
                      f"{self.embedded_data.redis.get(ROBOT_MULTICAST_CHANNEL_REDIS_KEY)}")
        table.add_row("Network Interface", f"{ROBOT_NETWORK_INTERFACE_REDIS_KEY}",
                      f"{self.embedded_data.redis.get(ROBOT_NETWORK_INTERFACE_REDIS_KEY)}")
        table.add_row("Kick Constant", f"{ROBOT_KICK_CONSTANT_REDIS_KEY}",
                      f"{self.embedded_data.redis.get(ROBOT_KICK_CONSTANT_REDIS_KEY)}")
        table.add_row("Kick Coefficient", f"{ROBOT_KICK_EXP_COEFF_REDIS_KEY}",
                      f"{self.embedded_data.redis.get(ROBOT_KICK_EXP_COEFF_REDIS_KEY)}")
        table.add_row("Chip Pulse Width", f"{ROBOT_CHIP_PULSE_WIDTH_REDIS_KEY}",
                      f"{self.embedded_data.redis.get(ROBOT_CHIP_PULSE_WIDTH_REDIS_KEY)}")
        table.add_row("Battery Voltage", f"{ROBOT_CURRENT_DRAW_REDIS_KEY}",
                      f"{self.embedded_data.redis.get(ROBOT_CURRENT_DRAW_REDIS_KEY)}")
        table.add_row("Capacitor Voltage", f"{ROBOT_BATTERY_VOLTAGE_REDIS_KEY}",
                      f"{self.embedded_data.redis.get(ROBOT_BATTERY_VOLTAGE_REDIS_KEY)}")
        return table

    def __clamp(self, val: float, min_val: float, max_val: float) -> float:
        """Simple Math Clamp function

        :param val: Value to clamp
        :param min_val: Minimum (Lower) Bound
        :param max_val: Maximum (Upper) Bound
        """
        # Faster than numpy & fewer dependencies
        return min(max(val, min_val), max_val)

    def stats(self) -> None:
        """CLI Command to generate Incoming RobotStatus Proto information"""
        with Live(self.__generate_stats_table(), refresh_per_second=4) as live:
            while True:
                live.update(self.__generate_stats_table())

    def redis(self):
        """CLI Command to generate Onboard Redis information"""
        self.console.print(self.__generate_redis_table())

    def status(self):
        """CLI Command to print Thunderloop service status"""
        log = subprocess.call(["service", "thunderloop", "status"])
        print(log)

    def restart_thunderloop(self):
        """CLI Command to restart Thunderloop service and print status"""
        subprocess.run(["service", "thunderloop", "restart"])
        self.status()

    def log(self):
        """CLI Command to print device logs"""
        log = subprocess.call(["sudo", "journalctl", "-f", "-n", "100"])
        print(log)

    def toggle_simple_mode(self):
        """Toggles Easy Shell Mode.

        - Easy Mode: Enables selection menu for functions
        - Regular Mode [Default]: Flags for functions
        """
        self.easy_mode_enabled = not self.easy_mode_enabled
        print(f"Easy Shell Mode set to {self.easy_mode_enabled}")

    @catch_interrupt_exception()
    def rotate(self,
               velocity: Annotated[Optional[float], Typer.Option(
                   help=f"Clamped to +{ROBOT_MAX_ANG_SPEED_RAD_PER_S} & -{ROBOT_MAX_ANG_SPEED_RAD_PER_S} rad/s")] = 0,
               duration_seconds: Annotated[Optional[float], Typer.Option(
                   help="Duration to rotate in seconds")] = DEFAULT_PRIMITIVE_DURATION
               ) -> None:
        """CLI Command to rotate the robot. Clamped by robot_max_ang_speed_rad_per_s.

        :param velocity: Angular Velocity to rotate the robot
        :param duration_seconds: Duration to rotate the robot
        """
        # TODO: Add InquirerPy with self.easy_mode_enabled
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
        description = f"Rotating at {velocity} rad/s for {duration_seconds} seconds"
        self.embedded_communication.run_primitive_over_time(duration_seconds, Primitive(direct_control=direct_control_primitive),
                                       description)

    @catch_interrupt_exception()
    def move(self,
             angle: Annotated[Optional[float], Typer.Option(
                 help=f"Direction to move in degrees")] = 90,
             speed: Annotated[Optional[float], Typer.Option(
                 help=f"Clamped to {0} & {ROBOT_MAX_SPEED_M_PER_S} m/s")] = 0,
             duration_seconds: Annotated[Optional[float], Typer.Option(
                 help="Duration to move in seconds")] = DEFAULT_PRIMITIVE_DURATION
             ) -> None:
        """CLI Command to move the robot in the specified direction

        :param angle: Direction to move in degrees
        :param speed: Speed to move the robot at
        :param duration_seconds: Duration to move
        """
        # TODO: Add InquirerPy with self.easy_mode_enabled
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
        description = f"Moving at {speed} m/s for {duration_seconds} seconds"
        self.embedded_communication.run_primitive_over_time(duration_seconds, Primitive(direct_control=direct_control_primitive),
                                       description)

    @catch_interrupt_exception()
    def chip(self,
             distance: Annotated[Optional[float], Typer.Option(
                 help=f"Distance to chip in meters")] = 0,
             auto: Annotated[Optional[bool], Typer.Option(
                 help=f"Enables auto chip (OVERWRITES CHIP DISTANCE)")] = False,
             duration_seconds: Annotated[Optional[float], Typer.Option(
                 help="Duration to be in chip mode in seconds")] = DEFAULT_PRIMITIVE_DURATION
             ) -> None:
        """CLI Command to chip for a specified distance
        :param distance: Distance to chip in meters
        :param auto: Enables auto chip (OVERWRITES CHIP DISTANCE)
        :param duration_seconds: Duration to be in chip mode in seconds
        """
        # TODO: Add InquirerPy with self.easy_mode_enabled
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
        description = f"Chipping {distance} m" if not auto else f"Auto Chip Enabled for {duration_seconds} seconds"
        self.embedded_communication.run_primitive_set(
            Primitive(direct_control=direct_control_primitive)
        )
        if auto:
            self.embedded_communication.run_primitive_over_time(duration_seconds, Primitive(direct_control=direct_control_primitive),
                                           description)

    @catch_interrupt_exception()
    def kick(self,
             speed: Annotated[Optional[float], Typer.Option(
                 help=f"Speed to kick in meters per second")] = 0,
             auto: Annotated[Optional[bool], Typer.Option(
                 help=f"Enables auto kick (OVERWRITES KICK SPEED)")] = False,
             duration_seconds: Annotated[Optional[float], Typer.Option(
                 help="Duration to be in kick mode in seconds")] = DEFAULT_PRIMITIVE_DURATION
             ) -> None:
        """CLI Command to kick at the specified speed
        :param speed: Speed to kick in meters per second
        :param auto: Enables auto kick (OVERWRITES KICK SPEED)
        :param duration_seconds: Duration to be in kick mode in seconds
        """
        # TODO: Add InquirerPy with self.easy_mode_enabled
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
        description = f"Kicking at {speed} m/s" if not auto else f"Auto Kick Enabled for {duration_seconds} seconds"
        self.embedded_communication.run_primitive_set(
            Primitive(direct_control=direct_control_primitive)
        )
        if auto:
            self.embedded_communication.run_primitive_over_time(duration_seconds, Primitive(direct_control=direct_control_primitive),
                                           description)

    @catch_interrupt_exception()
    def dribble(self,
                velocity: Annotated[Optional[float], Typer.Option(
                    help=f"Clamped to {-MAX_FORCE_DRIBBLER_SPEED_RPM} & {MAX_FORCE_DRIBBLER_SPEED_RPM} rpm")] = 0,
                duration_seconds: Annotated[Optional[float], Typer.Option(
                    help="Duration to move in seconds")] = DEFAULT_PRIMITIVE_DURATION
                ) -> None:
        """CLI Command to rotate the robot's dribbler in the specified rpm

        :param velocity: Speed & Direction to rotate the dribbler
        :param duration_seconds: Duration to rotate
        """
        # TODO: Add InquirerPy with self.easy_mode_enabled
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
        description = f"Spinning dribbler at {velocity} rpm for {duration_seconds} seconds"
        self.embedded_communication.run_primitive_over_time(duration_seconds, Primitive(direct_control=direct_control_primitive),
                                       description)

    @catch_interrupt_exception()
    def move_wheel(self,
                   wheels: Annotated[List[int], Typer.Argument(
                       help=f'Wheel to rotate {{1:"NE", 2:"SE", 3:"SW", 4:"NW"}}')],
                   velocity: Annotated[Optional[float], Typer.Option(
                       help=f"Clamped to {-ROBOT_MAX_SPEED_M_PER_S} & {ROBOT_MAX_SPEED_M_PER_S} m/s")] = 0,
                   duration_seconds: Annotated[Optional[float], Typer.Option(
                       help="Duration to move in seconds")] = DEFAULT_PRIMITIVE_DURATION
                   ) -> None:
        """CLI Command to move the robot in the specified direction

        :param wheels: Wheels to rotate
        :param velocity: Velocity to rotate the wheel
        :param duration_seconds: Duration to move
        """

        # TODO: Add InquirerPy with self.easy_mode_enabled
        # TODO: Confirm max speed for wheel rotation (it is currently net robot velocity)
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
        description = f"Moving wheels {wheels} at {velocity} m/s for {duration_seconds} seconds"
        self.embedded_communication.run_primitive_over_time(duration_seconds, Primitive(direct_control=direct_control_primitive),
                                       description)

    def emote(self):
        # TODO: Add an emote function!
        pass



if __name__ == "__main__":
    with EmbeddedCommunication() as embedded_communication:
        RobotDiagnosticsCLI(embedded_communication).app()
