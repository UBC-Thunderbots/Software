import math

import typer
from rich import print
from rich.live import Live
from rich.table import Table
from rich.console import Console
from rich.progress import track
import software.python_bindings as tbots_cpp
from software.py_constants import *
from typer_shell import make_typer_shell
from google.protobuf.message import Message
from software.embedded.constants.py_constants import *
from proto.import_all_protos import *
import redis
import subprocess
import InquirerPy
import time
import typer as Typer
from functools import wraps
from typing import Optional
from typing_extensions import Annotated


class RobotDiagnosticsCLI:
    def __init__(self) -> None:
        """Setup constructor for the Shell CLI"""
        self.app = make_typer_shell(prompt="⚡ ")
        self.app.command(short_help="Rotates the robot")(self.rotate)
        self.app.command(short_help="Moves the robot")(self.move)
        self.app.command(short_help="Chips the chipper")(self.chip)
        self.app.command(short_help="Kicks the kicker")(self.kick)
        self.app.command(short_help="Show Robot Status Info")(self.stats)
        self.app.command(short_help="Spins the dribbler")(self.dribble)
        self.app.command(short_help="Restarts Thunderloop")(self.restart_thunderloop)
        self.app.command(short_help="Shows Redis Values")(self.redis)
        self.app.command(short_help="Prints Thunderloop Logs")(self.log)
        self.app.command(short_help="Prints Thunderloop Status")(self.status)
        self.app.command(short_help="Toggles easy selection shell (For new users)")(self.toggle_simple_mode)

        self.redis = redis.StrictRedis(
            host=REDIS_DEFAULT_HOST,
            port=REDIS_DEFAULT_PORT,
            charset="utf-8",
            decode_responses=True
        )
        self.channel_id = int(self.redis.get(ROBOT_ID_REDIS_KEY))
        self.robot_id = int(self.redis.get(ROBOT_ID_REDIS_KEY))
        self.sequence_number = 0
        self.console = Console()
        self.command_duration_seconds = 2.0
        self.send_primitive_interval_s = 0.01
        self.easy_mode_enabled = False
        # total primitives for all robots
        self.robot_primitives = {}

        # for all robots connected to diagnostics, set their primitive
        self.robot_primitives[self.robot_id] = Primitive(stop=StopPrimitive())

        # initialising the estop
        # tries to access a plugged in estop. if not found, throws an exception
        # if using keyboard estop, skips this step
        self.estop_reader = None
        self.estop_is_playing = False
        # when the estop has just been stopped,
        # we want to send a stop primitive once to all currently connected robots
        self.should_send_stop = False
        self.estop_mode, self.estop_path = (
            get_estop_config(keyboard_estop=True, disable_communication=False))
        # only checks for estop if we are in physical estop mode
        if self.estop_mode == EstopMode.PHYSICAL_ESTOP:
            try:
                self.estop_reader = tbots_cpp.ThreadedEstopReader(
                    uart_port=self.estop_path, baud_rate=115200
                )
            except Exception:
                raise Exception(f"Invalid Estop found at location {self.estop_path}")

    def __should_send_packet(self) -> bool:
        """Returns True if the proto sending threads should send a proto

        :return: boolean
        """
        return ((self.estop_mode != EstopMode.DISABLE_ESTOP) and self.estop_is_playing)

    def __run_primitive_set(self, diagnostics_primitive: Primitive) -> None:
        """Forward PrimitiveSet protos from diagnostics to the robots.

        For Diagnostics protos, does not block and returns cached message if none available
        Sleeps for 10ms for diagnostics

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.
        """

        # for all robots connected to diagnostics, set their primitive

        self.robot_primitives[self.robot_id] = diagnostics_primitive
        # initialize total primitive set and send it
        primitive_set = PrimitiveSet(
            time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
            stay_away_from_ball=False,
            robot_primitives=(
                self.robot_primitives
                if not self.should_send_stop
                else {
                    robot_id: Primitive(stop=StopPrimitive())
                    for robot_id in self.robot_primitives.keys()
                }
            ),
            sequence_number=self.sequence_number,
        )

        self.sequence_number += 1

        # if self.__should_send_packet() or self.should_send_stop:
        self.send_primitive_set.send_proto(primitive_set)
        self.should_send_stop = False

    def catch_interrupt_exception(exit_code=1):
        """Decorator for handling keyboard exceptions and safely clearing cached primitives"""

        def decorator(func):
            @wraps(func)
            def wrapper(self, *args, **kwargs):
                try:
                    return func(self, *args, **kwargs)
                except KeyboardInterrupt:
                    print("Stopped Primitive Send")
                    self.__run_primitive_set(Primitive(stop=StopPrimitive()))
                    raise Typer.Exit(code=exit_code)

            return wrapper

        return decorator

    def __send_estop_state(self) -> None:
        """Constant loop which sends the current estop status proto if estop is not disabled
        Always in physical estop mode, uses the physical estop value
        If estop has just changed from playing to stop, set flag to send stop primitive once to connected robots
        """
        previous_estop_is_playing = True
        if self.estop_mode != EstopMode.DISABLE_ESTOP:
            while True:
                if self.estop_mode == EstopMode.PHYSICAL_ESTOP:
                    self.estop_is_playing = self.estop_reader.isEstopPlay()

                # Send stop primitive once when estop is paused
                if previous_estop_is_playing and not self.estop_is_playing:
                    self.should_send_stop = True
                else:
                    self.should_send_stop = False

                previous_estop_is_playing = self.estop_is_playing
                time.sleep(0.1)

    def __receive_robot_status(self, robot_status: Message) -> None:
        """Forwards the given robot status to the full system along with the round-trip time

        :param robot_status: RobotStatus to extract information from
        """
        self.epoch_timestamp_seconds = robot_status.time_sent.epoch_timestamp_seconds
        self.battery_voltage = robot_status.power_status.battery_voltage
        self.primitive_packet_loss_percentage = robot_status.network_status.primitive_packet_loss_percentage
        self.primitive_executor_step_time_ms = robot_status.thunderloop_status.primitive_executor_step_time_ms

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
            f"{self.redis.get(ROBOT_ID_REDIS_KEY)}",
            f"{self.battery_voltage}",
            status,
            f"{self.epoch_timestamp_seconds}"
        )
        return table

    def __generate_redis_table(self) -> Table:
        """Make a new table with redis value information."""
        table = Table(show_header=True, header_style="bold blue")
        table.add_column("Redis Value Name")
        table.add_column("Key", style="dim")
        table.add_column("Value")

        table.add_row("Robot ID", f"{ROBOT_ID_REDIS_KEY}",
                      f"{self.redis.get(ROBOT_ID_REDIS_KEY)}")
        table.add_row("Channel ID", f"{ROBOT_MULTICAST_CHANNEL_REDIS_KEY}",
                      f"{self.redis.get(ROBOT_MULTICAST_CHANNEL_REDIS_KEY)}")
        table.add_row("Network Interface", f"{ROBOT_NETWORK_INTERFACE_REDIS_KEY}",
                      f"{self.redis.get(ROBOT_NETWORK_INTERFACE_REDIS_KEY)}")
        table.add_row("Kick Constant", f"{ROBOT_KICK_CONSTANT_REDIS_KEY}",
                      f"{self.redis.get(ROBOT_KICK_CONSTANT_REDIS_KEY)}")
        table.add_row("Kick Coefficient", f"{ROBOT_KICK_EXP_COEFF_REDIS_KEY}",
                      f"{self.redis.get(ROBOT_KICK_EXP_COEFF_REDIS_KEY)}")
        table.add_row("Chip Pulse Width", f"{ROBOT_CHIP_PULSE_WIDTH_REDIS_KEY}",
                      f"{self.redis.get(ROBOT_CHIP_PULSE_WIDTH_REDIS_KEY)}")
        table.add_row("Battery Voltage", f"{ROBOT_CURRENT_DRAW_REDIS_KEY}",
                      f"{self.redis.get(ROBOT_CURRENT_DRAW_REDIS_KEY)}")
        table.add_row("Capacitor Voltage", f"{ROBOT_BATTERY_VOLTAGE_REDIS_KEY}",
                      f"{self.redis.get(ROBOT_BATTERY_VOLTAGE_REDIS_KEY)}")
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
               velocity: Annotated[Optional[float], typer.Option(
                   help=f"Clamped to +{ROBOT_MAX_ANG_SPEED_RAD_PER_S} & -{ROBOT_MAX_ANG_SPEED_RAD_PER_S} rad/s")] = 0,
               duration_seconds: Annotated[Optional[float], typer.Option(
                   help="Duration to rotate in seconds")] = DEFAULT_PRIMITIVE_DURATION
               ) -> None:
        """CLI Command to rotate the robot. Clamped by robot_max_ang_speed_rad_per_s.

        :param velocity: Angular Velocity to rotate the robot
        :param duration_seconds: Duration to rotate the robot
        """
        # TODO: Add InquirerPy with self.easy_mode_enabled
        self.__run_primitive_set(Primitive(stop=StopPrimitive()))
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
        for _ in track(range(int(duration_seconds / self.send_primitive_interval_s)),
                       description=f"Rotating at {velocity} rad/s for {duration_seconds} seconds"):
            self.__run_primitive_set(
                Primitive(direct_control=direct_control_primitive)
            )
            time.sleep(self.send_primitive_interval_s)
        self.__run_primitive_set(Primitive(stop=StopPrimitive()))

    @catch_interrupt_exception()
    def move(self,
             angle: Annotated[Optional[float], typer.Option(
                 help=f"Direction to move in degrees")] = 90,
             speed: Annotated[Optional[float], typer.Option(
                 help=f"Clamped to {0} & {ROBOT_MAX_SPEED_M_PER_S} m/s")] = 0,
             duration_seconds: Annotated[Optional[float], typer.Option(
                 help="Duration to move in seconds")] = DEFAULT_PRIMITIVE_DURATION
             ) -> None:
        """CLI Command to move the robot in the specified direction

        :param angle: Direction to move
        :param speed: Speed to move the robot at
        :param duration_seconds: Duration to move
        """
        # TODO: Add InquirerPy with self.easy_mode_enabled
        self.__run_primitive_set(Primitive(stop=StopPrimitive()))
        speed = self.__clamp(
            val=speed,
            min_val=0,
            max_val=ROBOT_MAX_SPEED_M_PER_S
        )
        motor_control_primitive = MotorControl()
        motor_control_primitive.direct_velocity_control.velocity = Vector(
            x_component_meters=speed * math.cos(angle),
            y_component_meters=speed * math.sin(angle)
        )
        direct_control_primitive = DirectControlPrimitive(
            motor_control=motor_control_primitive,
            power_control=PowerControl()
        )
        for _ in track(range(int(duration_seconds / self.send_primitive_interval_s)),
                       description=f"Moving at {speed} rad/s for {duration_seconds} seconds"):
            self.__run_primitive_set(
                Primitive(direct_control=direct_control_primitive)
            )
            time.sleep(self.send_primitive_interval_s)
        self.__run_primitive_set(Primitive(stop=StopPrimitive()))

    def chip(self, distance_meter: float = 1.0) -> None:
        distance_meter = self.__clamp(distance_meter, 0, 2.0)
        print(f"Chipping {distance_meter} meters")

    def kick(self, speed_m_per_s: float = 2.0) -> None:
        speed_m_per_s = self.__clamp(speed_m_per_s, 0, 6.0)
        print(f"Kicking at {speed_m_per_s} meters per second")

    def dribble(self, velocity_rad_per_s: float) -> None:
        velocity_rad_per_s = self.__clamp(velocity_rad_per_s, 0, 5.0)
        print(f"Spinning dribbler at {velocity_rad_per_s} rad per second")

    def move_wheel(self):
        # py_inquire wheel choice or use sub shell
        pass

    def emote(self):
        pass

    def __enter__(self):
        """Enter RobotDiagnosticsCLI context manager. Setup multicast listeners
        for RobotStatus, RobotLogs, and RobotCrash msgs, and multicast sender for PrimitiveSet
        """
        # Receiver
        self.receive_robot_status = tbots_cpp.RobotStatusProtoListener(
            str(getRobotMulticastChannel(
                self.channel_id)) + "%" + f"{self.redis.get(ROBOT_NETWORK_INTERFACE_REDIS_KEY)}",
            ROBOT_STATUS_PORT,
            self.__receive_robot_status,
            True,
        )

        # Sender
        self.send_primitive_set = tbots_cpp.PrimitiveSetProtoUdpSender(
            str(getRobotMulticastChannel(
                self.channel_id)) + "%" + f"{self.redis.get(ROBOT_NETWORK_INTERFACE_REDIS_KEY)}", PRIMITIVE_PORT, True
        )

        return self

    def __exit__(self, type, value, traceback) -> None:
        """Exit RobotDiagnosticsCLI context manager

        Ends all currently running loops and joins all currently active threads
        """
        self.receive_robot_status.close()


if __name__ == "__main__":
    with RobotDiagnosticsCLI() as cli:
        cli.app()
