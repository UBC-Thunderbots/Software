import subprocess
import logging
import typer as Typer
from rich import print
from rich.live import Live
from rich.table import Table
from rich.console import Console
from rich.logging import RichHandler
from software.py_constants import *
from typer_shell import make_typer_shell
from functools import wraps
from typing import List, Optional
from typing_extensions import Annotated
from embedded_communication import EmbeddedCommunication
from proto.import_all_protos import *
from software.embedded.constants.py_constants import (
    DEFAULT_PRIMITIVE_DURATION,
    ROBOT_MAX_ANG_SPEED_RAD_PER_S,
    ROBOT_MAX_SPEED_M_PER_S,
    MAX_FORCE_DRIBBLER_SPEED_RPM,
)


class RobotDiagnosticsCLI:
    """Onboard lightweight Diagnostics CLI interface running on the robots. UI and Main class"""

    def __init__(self, embedded_communication: EmbeddedCommunication) -> None:
        """Setup constructor for the Shell CLI
        :param embedded_communication: Communication object with open connection to robots for sending protos
        """
        intro = """[bold yellow]Welcome to the RobotDiagnostics CLI [/bold yellow]
        
        Type [bold blue]help[/bold blue] for a list of commands. 
        Use [bold blue]--help[/bold blue] in commands for extra info. 
        If you ever have questions feel free to reach out!"""

        self.app = make_typer_shell(
            prompt="⚡ ", launch=lambda x: print(intro), intro="\n"
        )
        self.app.command(short_help="Moves the robot")(self.move)
        self.app.command(short_help="Moves specific wheels of the robot")(
            self.move_wheel
        )
        self.app.command(short_help="Rotates the robot")(self.rotate)
        self.app.command(short_help="Spins the dribbler")(self.dribble)
        self.app.command(short_help="Chips the chipper")(self.chip)
        self.app.command(short_help="Kicks the kicker")(self.kick)
        self.app.command(short_help="Show Robot Status Info")(self.stats)
        self.app.command(short_help="Shows Yaml Values")(self.yaml)
        self.app.command(short_help="Prints Thunderloop Logs")(self.log)
        self.app.command(short_help="Prints Thunderloop Status")(self.status)
        self.app.command(short_help="Restarts Thunderloop")(self.restart_thunderloop)
        # Communication object responsible for proto execution/transmission
        self.embedded_communication = embedded_communication
        # Data handler responsible for disk information
        self.embedded_data = self.embedded_communication.embedded_data

        self.console = Console()
        logging.basicConfig(
            level=logging.INFO,
            handlers=[RichHandler(rich_tracebacks=True, markup=True)],
        )

    def catch_interrupt_exception(exit_code=1):
        """Decorator for handling keyboard exceptions and safely clearing cached primitives.
        This special function allows us to generalize catching repeated expected Errors without
        try-catches in every function
        """

        def decorator(func):
            @wraps(func)
            def wrapper(self, *args, **kwargs):
                try:
                    self.embedded_communication.send_primitive(
                        Primitive(stop=StopPrimitive())
                    )
                    return func(self, *args, **kwargs)
                except KeyboardInterrupt:
                    logging.info(
                        "[bold yellow] E-Stop Activated: Stopped Primitive Send [/bold yellow]"
                    )
                    self.embedded_communication.send_primitive(
                        Primitive(stop=StopPrimitive())
                    )
                    raise Typer.Exit(code=exit_code)
                except Exception as e:
                    self.embedded_communication.send_primitive(
                        Primitive(stop=StopPrimitive())
                    )
                    logging.exception(f"Unknown Exception: {e}")
                    raise Typer.Exit(code=exit_code)
                finally:
                    self.embedded_communication.send_primitive(
                        Primitive(stop=StopPrimitive())
                    )

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
        if (
            subprocess.call(
                ["systemctl", "is-active", "--quiet", "thunderloop.service"]
            )
            == 0
        ):
            status = "[green]ONLINE"

        table.add_row(
            self.embedded_data.get_robot_id(),
            f"{self.embedded_data.battery_voltage}",
            status,
            f"{self.embedded_data.epoch_timestamp_seconds}",
        )
        return table

    def __generate_yaml_table(self) -> Table:
        """Make a new table with embedded_data.redis value information."""
        table = Table(show_header=True, header_style="bold blue")
        table.add_column("Redis Value Name")
        table.add_column("Key", style="dim")
        table.add_column("Value")

        table.add_row(
            "Robot ID", f"{ROBOT_ID_YAML_KEY}", self.embedded_data.get_robot_id()
        )
        table.add_row(
            "Channel ID",
            f"{ROBOT_MULTICAST_CHANNEL_YAML_KEY}",
            self.embedded_data.get_channel_id(),
        )
        table.add_row(
            "Network Interface",
            f"{ROBOT_NETWORK_INTERFACE_YAML_KEY}",
            self.embedded_data.get_network_interface(),
        )
        table.add_row(
            "Kick Constant",
            f"{ROBOT_KICK_CONSTANT_YAML_KEY}",
            self.embedded_data.get_kick_constant(),
        )
        table.add_row(
            "Kick Coefficient",
            f"{ROBOT_KICK_EXP_COEFF_YAML_KEY}",
            self.embedded_data.get_kick_coeff(),
        )
        table.add_row(
            "Chip Pulse Width",
            f"{ROBOT_CHIP_PULSE_WIDTH_YAML_KEY}",
            self.embedded_data.get_chip_pulse_width(),
        )
        return table

    def stats(self) -> None:
        """CLI Command to generate Incoming RobotStatus Proto information"""
        with Live(self.__generate_stats_table(), refresh_per_second=4) as live:
            while True:
                live.update(self.__generate_stats_table())

    def yaml(self):
        """CLI Command to generate Onboard Yaml information"""
        self.console.print(self.__generate_yaml_table())

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

    @catch_interrupt_exception()
    def rotate(
        self,
        velocity: Annotated[
            Optional[float],
            Typer.Option(
                help=f"Clamped to +{ROBOT_MAX_ANG_SPEED_RAD_PER_S} & -{ROBOT_MAX_ANG_SPEED_RAD_PER_S} rad/s"
            ),
        ] = 0,
        duration_seconds: Annotated[
            Optional[float], Typer.Option(help="Duration to rotate in seconds")
        ] = DEFAULT_PRIMITIVE_DURATION,
    ) -> None:
        """CLI Command to rotate the robot. Clamped by robot_max_ang_speed_rad_per_s.
        :param velocity: Angular Velocity to rotate the robot
        :param duration_seconds: Duration to rotate the robot
        """
        description = f"Rotating at {velocity} rad/s for {duration_seconds} seconds"
        self.embedded_communication.run_primitive_over_time(
            duration_seconds,
            self.embedded_data.get_rotate_primitive(velocity),
            description,
        )

    @catch_interrupt_exception()
    def move(
        self,
        angle: Annotated[
            Optional[float], Typer.Option(help="Direction to move in degrees")
        ] = 90,
        speed: Annotated[
            Optional[float],
            Typer.Option(help=f"Clamped to {0} & {ROBOT_MAX_SPEED_M_PER_S} m/s"),
        ] = 0,
        duration_seconds: Annotated[
            Optional[float], Typer.Option(help="Duration to move in seconds")
        ] = DEFAULT_PRIMITIVE_DURATION,
    ) -> None:
        """CLI Command to move the robot in the specified direction

        :param angle: Direction to move in degrees
        :param speed: Speed to move the robot at
        :param duration_seconds: Duration to move
        """
        description = (
            f"Moving at {speed} m/s for {duration_seconds} seconds towards {angle}"
        )
        self.embedded_communication.run_primitive_over_time(
            duration_seconds,
            self.embedded_data.get_move_primitive(angle, speed),
            description,
        )

    @catch_interrupt_exception()
    def chip(
        self,
        distance: Annotated[
            Optional[float], Typer.Option(help="Distance to chip in meters")
        ] = 0,
        auto: Annotated[
            Optional[bool],
            Typer.Option(help="Enables auto chip (OVERWRITES CHIP DISTANCE)"),
        ] = False,
        duration_seconds: Annotated[
            Optional[float], Typer.Option(help="Duration to be in chip mode in seconds")
        ] = DEFAULT_PRIMITIVE_DURATION,
    ) -> None:
        """CLI Command to chip for a specified distance
        :param distance: The distance to chip in meters
        :param auto: Enables auto chip (OVERWRITES CHIP DISTANCE)
        :param duration_seconds: Duration to be in chip mode in seconds
        """
        description = (
            f"Chipping {distance} m"
            if not auto
            else f"Auto Chip Enabled for {duration_seconds} seconds"
        )
        primitive = self.embedded_data.get_kick_primitive(auto, distance)
        if auto:
            self.embedded_communication.run_primitive_over_time(
                duration_seconds, primitive, description
            )
        else:
            self.embedded_communication.run_primitive(primitive)
            print(description)

    @catch_interrupt_exception()
    def kick(
        self,
        speed: Annotated[
            Optional[float], Typer.Option(help="Speed to kick in meters per second")
        ] = 0,
        auto: Annotated[
            Optional[bool],
            Typer.Option(help="Enables auto kick (OVERWRITES KICK SPEED)"),
        ] = False,
        duration_seconds: Annotated[
            Optional[float], Typer.Option(help="Duration to be in kick mode in seconds")
        ] = DEFAULT_PRIMITIVE_DURATION,
    ) -> None:
        """CLI Command to kick at the specified speed
        :param speed: The speed to kick in meters per second
        :param auto: Enables auto kick (OVERWRITES KICK SPEED)
        :param duration_seconds: Duration to be in kick mode in seconds
        """
        description = (
            f"Kicking at {speed} m/s"
            if not auto
            else f"Auto Kick Enabled for {duration_seconds} seconds"
        )
        primitive = self.embedded_data.get_kick_primitive(auto, speed)
        if auto:
            self.embedded_communication.run_primitive_over_time(
                duration_seconds, primitive, description
            )
        else:
            zero_direct_control_primitive = DirectControlPrimitive(
                motor_control=self.embedded_data.get_zero_motor_control_primitive(),
                power_control=self.embedded_data.get_zero_power_control_primitive(),
            )
            self.embedded_communication.run_primitive(primitive)
            print(description)
            self.embedded_communication.run_primitive_over_time(
                1,
                Primitive(direct_control=zero_direct_control_primitive),
                "Recharging...",
            )

    @catch_interrupt_exception()
    def dribble(
        self,
        velocity: Annotated[
            Optional[float],
            Typer.Option(
                help=f"Clamped to {-MAX_FORCE_DRIBBLER_SPEED_RPM} & {MAX_FORCE_DRIBBLER_SPEED_RPM} rpm"
            ),
        ] = 0,
        duration_seconds: Annotated[
            Optional[float], Typer.Option(help="Duration to move in seconds")
        ] = DEFAULT_PRIMITIVE_DURATION,
    ) -> None:
        """CLI Command to rotate the robot's dribbler in the specified rpm

        :param velocity: Speed & Direction to rotate the dribbler
        :param duration_seconds: Duration to rotate
        """
        description = (
            f"Spinning dribbler at {velocity} rpm for {duration_seconds} seconds"
        )
        self.embedded_communication.run_primitive_over_time(
            duration_seconds,
            self.embedded_data.get_dribble_primitive(velocity),
            description,
        )

    @catch_interrupt_exception()
    def move_wheel(
        self,
        wheels: Annotated[
            List[int],
            Typer.Argument(help='Wheel to rotate {1:"NE", 2:"SE", 3:"SW", 4:"NW"}'),
        ],
        velocity: Annotated[
            Optional[float],
            Typer.Option(
                help=f"Clamped to {-ROBOT_MAX_SPEED_M_PER_S} & {ROBOT_MAX_SPEED_M_PER_S} m/s"
            ),
        ] = 0,
        duration_seconds: Annotated[
            Optional[float], Typer.Option(help="Duration to move in seconds")
        ] = DEFAULT_PRIMITIVE_DURATION,
    ) -> None:
        """CLI Command to move the robot in the specified direction

        :param wheels: Wheels to rotate
        :param velocity: Velocity to rotate the wheel
        :param duration_seconds: Duration to move
        """
        # TODO (#3436): Confirm max speed for wheel rotation (it is currently net robot velocity)
        description = (
            f"Moving wheels {wheels} at {velocity} m/s for {duration_seconds} seconds"
        )
        self.embedded_communication.run_primitive_over_time(
            duration_seconds,
            self.embedded_data.get_move_wheel_primitive(wheels, velocity),
            description,
        )

    def emote(self):
        # TODO (#3434): Add an emote function!
        return


if __name__ == "__main__":
    with EmbeddedCommunication() as embedded_communication:
        RobotDiagnosticsCLI(embedded_communication).app()
