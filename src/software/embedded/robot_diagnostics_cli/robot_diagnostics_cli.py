from rich import print
from rich.live import Live
from rich.table import Table
# import software.python_bindings as tbots_cpp
# from software.py_constants import *
from typer_shell import make_typer_shell
from google.protobuf.message import Message

class RobotDiagnosticsCLI():
    def __init__(self) -> None:
        self.app = make_typer_shell(prompt="⚡ ")
        self.app.command(short_help="Rotates the robot")(self.rotate)
        self.app.command(short_help="Moves the robot")(self.move)
        self.app.command(short_help="Chips the chipper")(self.chip)
        self.app.command(short_help="Kicks the kicker")(self.kick)
        self.app.command(short_help="Show Robot stats")(self.stats)
        self.app.command(short_help="Spins the dribbler")(self.dribble)
        self.app.command(short_help="Restarts Thunderloop")(self.restart_thunderloop)

        self.test_val = 0

        # Receiver / probably want to fetch channel from redis cache
        self.receive_robot_status = tbots_cpp.RobotStatusProtoListener(
            str(getRobotMulticastChannel(0)) + "%" + "eth0",
            ROBOT_STATUS_PORT,
            self.__receive_robot_status,
            True,
            )

        # Sender / What is the network interface here?
        self.send_primitive_set = tbots_cpp.PrimitiveSetProtoUdpSender(
            str(getRobotMulticastChannel(0)) + "%" + "eth0", PRIMITIVE_PORT, True
        )

    def __receive_robot_status(self, robot_status: Message) -> None:
        """Forwards the given robot status to the full system along with the round-trip time

        :param robot_status: RobotStatus to forward to fullsystem
        """
        # round_trip_time_seconds = time.time() - (
        #     robot_status.adjusted_time_sent.epoch_timestamp_seconds
        # )
        # self.__forward_to_proto_unix_io(
        #     RobotStatistic,
        #     RobotStatistic(round_trip_time_seconds=round_trip_time_seconds),
        # )
        # self.__forward_to_proto_unix_io(RobotStatus, robot_status)
        print(f"Received: {robot_status.adjusted_time_sent.epoch_timestamp_seconds}")
        self.test_val = robot_status.adjusted_time_sent.epoch_timestamp_seconds

    def __clamp(self, num: float, min_val: float, max_val: float) -> float:
        return max(min(num, max_val), min_val)

    def __generate_table(self) -> Table:
        """Make a new table."""
        table = Table()
        table.add_column("Robot ID")
        table.add_column("Status")
        table.add_column("Battery (V)")

        value = self.test_val
        status = ""
        if value < 50:
            status = "[red]OFFLINE"
        else:
            status = "[green]ONLINE"
        table.add_row(
            f"{0}", f"{value:3.2f}", status
        )
        return table

    def rotate(self, velocity_in_rad: float) -> None:
        # CLAMP SPEED
        MAX_SPEED_RAD = 4
        velocity_in_rad = self.__clamp(velocity_in_rad, -MAX_SPEED_RAD, MAX_SPEED_RAD)
        print(f"Rotating at {velocity_in_rad} rad/s")

    def move(self, direction: str, speed: float) -> None:
        default_commands: dict = {
            "forward": 90,
            "back": 270,
            "left": 180,
            "right": 0
        }
        direction = direction.strip()
        direction = direction.lower()
        # CLAMP SPEED
        MAX_VALUE = 100
        MIN_VALUE = 0
        speed = self.__clamp(speed, MIN_VALUE, MAX_VALUE)
        if direction in default_commands:
            print(f"Going {direction} and mapping to {default_commands[direction]} at the current speed {speed}")
        else:
            print("ERROR: INVALID COMMAND")

    def chip(self, distance_meter: float = 1.0) -> None:
        distance_meter = self.__clamp(distance_meter, 0, 2.0)
        print(f"Chipping {distance_meter} meters")

    def kick(self, speed_m_per_s: float = 2.0) -> None:
        speed_m_per_s = self.__clamp(speed_m_per_s, 0, 6.0)
        print(f"Kicking at {speed_m_per_s} meters per second")

    def stats(self) -> None:
        with Live(self.__generate_table(), refresh_per_second=4) as live:
            while True:
                live.update(self.__generate_table())

    def dribble(self, velocity_rad_per_s: float) -> None:
        velocity_rad_per_s = self.__clamp(velocity_rad_per_s, 0, 5.0)
        print(f"Spinning dribbler at {velocity_rad_per_s} rad per second")

    def restart_thunderloop(self):
        # Execute some bash command
        pass

    def emote(self):
        pass
    def move_wheel(self):
        # py_inquire wheel choice or use sub shell
        pass


if __name__ == "__main__":
    while True:
        RobotDiagnosticsCLI().app()

