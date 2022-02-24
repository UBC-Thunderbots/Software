from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.networking.threaded_unix_sender import ThreadedUnixSender
from proto.sensor_msg_pb2 import SensorProto
from proto.world_pb2 import WorldState, SimulatorTick
from proto.vision_pb2 import RobotState, BallState
from proto.robot_status_msg_pb2 import RobotStatus
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.geometry_pb2 import Point, Angle, Vector, AngularVelocity
from subprocess import Popen, PIPE
import subprocess


class StandaloneSimulatorWrapper(object):
    def __init__(self):
        """Runs our standalone er-force simulator binary and sets up the unix
        sockets to communicate with it

        """
        self.sim_tick_sender = ThreadedUnixSender("/tmp/tbots/simulation_tick")
        self.world_state_sender = ThreadedUnixSender("/tmp/tbots/world_state")
        self.blue_vision_sender = ThreadedUnixSender("/tmp/tbots/blue_vision")
        self.yellow_vision_sender = ThreadedUnixSender("/tmp/tbots/yellow_vision")
        self.blue_primitive_set_sender = ThreadedUnixSender(
            "/tmp/tbots/blue_primitive_set"
        )
        self.yellow_primitive_set_sender = ThreadedUnixSender(
            "/tmp/tbots/yellow_primitive_set"
        )

        self.ssl_wrapper_listener = ThreadedUnixListener(
            "/tmp/tbots/ssl_wrapper_packet", SSL_WrapperPacket, convert_from_any=False
        )
        self.blue_robot_status_listener = ThreadedUnixListener(
            "/tmp/tbots/blue_robot_status", RobotStatus, convert_from_any=False
        )
        self.yellow_robot_status_listener = ThreadedUnixListener(
            "/tmp/tbots/yellow_robot_status", RobotStatus, convert_from_any=False
        )
        self.world_state = WorldState()

        # TODO change to er_force_sim_main
        self.standalone_simulator = Popen(
            ["software/simulation/standalone_er_force_simulator_main"],
            stdout=PIPE,
            stderr=PIPE,
        )

    def __setup_robots(self, robot_locations, team_colour):
        """Initializes the world from a list of robot locations

        :param robot_locations: A list of robot locations (index is robot id)
        :param team_colour: The color (either "blue" or "yellow")

        """

        if "blue" in team_colour:
            robot_map = self.world_state.blue_robots
        else:
            robot_map = self.world_state.yellow_robots

        for robot_id, robot_location in enumerate(robot_locations):
            robot_map[robot_id].CopyFrom(
                RobotState(
                    global_position=Point(
                        x_meters=robot_location[0], y_meters=robot_location[1]
                    ),
                    global_orientation=Angle(radians=0),
                    global_velocity=Vector(x_component_meters=0, y_component_meters=0),
                    global_angular_velocity=AngularVelocity(radians_per_second=0),
                )
            )

        self.setup_world(self.world_state)

    def setup_blue_robots(self, robot_locations):
        """Initializes the world from a list of robot locations

        :param robot_locations: A list of robot locations (index is robot id)

        """
        self.__setup_robots(robot_locations, "blue")

    def setup_yellow_robots(self, robot_locations):
        """Initializes the world from a list of robot locations

        :param robot_locations: A list of robot locations (index is robot id)

        """
        self.__setup_robots(robot_locations, "yellow")

    def setup_ball(
        self, ball_position=(0, 0), ball_velocity=(0, 0), distance_from_ground=0
    ):
        """Setup the ball with the x, y coordinates in meters

        :param ball_position: A tuple with the x,y coordinates
        :param ball_velocity: A tuple with the x,y velocity components
        :param distance_from_ground: How high up to start the ball

        """
        self.world_state.ball_state.CopyFrom(
            BallState(
                global_position=Point(
                    x_meters=ball_position[0], y_meters=ball_position[1]
                ),
                global_velocity=Vector(
                    x_component_meters=ball_velocity[0],
                    y_component_meters=ball_velocity[1],
                ),
                distance_from_ground=distance_from_ground,
            )
        )

        self.setup_world(self.world_state)

    def setup_world(self, world_state):
        """Pass in a world_state proto directly to setup the simulator

        :param world_state: The world state to initialize with

        """
        self.world_state_sender.send(world_state)

    def __get_sensor_proto(self, ssl_wrapper, robot_status_listener):
        """Helper function to create a sensor proto

        :param ssl_wrapper: The ssl_wrapper packet to put in the sensor proto
        :param robot_status_listener: The robot status listener (blue or yellow)
        :returns: A sensor proto with the robot status from the listener

        """
        sensor_proto = SensorProto()
        sensor_proto.ssl_vision_msg.CopyFrom(ssl_wrapper)

        robot_status = robot_status_listener.maybe_pop()

        packets = []
        while robot_status is not None:
            packets.append(robot_status)
            robot_status = robot_status_listener.maybe_pop()

        sensor_proto.robot_status_msgs.extend(packets)
        return sensor_proto

    def get_blue_sensor_proto(self, ssl_wrapper):
        return self.__get_sensor_proto(ssl_wrapper, self.blue_robot_status_listener)

    def get_yellow_sensor_proto(self, ssl_wrapper):
        return self.__get_sensor_proto(ssl_wrapper, self.yellow_robot_status_listener)

    def get_ssl_wrapper_packet(self):
        return self.ssl_wrapper_listener.maybe_pop()

    def tick(self, duration_ms):
        """Tick the simulator with the given duration

        :param duration_ms: The duration to step the sim

        """
        tick = SimulatorTick()
        tick.milliseconds = duration_ms
        self.sim_tick_sender.send(tick)

    def send_blue_primitive_set_and_vision(self, vision, primitive_set):
        """Blue primitive set and vision

        :param vision: The vision msg to send
        :param primitive_set: The primitive set to send

        """
        self.blue_vision_sender.send(vision)
        self.blue_primitive_set_sender.send(primitive_set)

    def send_yellow_primitive_set_and_vision(self, vision, primitive_set):
        """Yellow primitive set and vision

        :param vision: The vision msg to send
        :param primitive_set: The primitive set to send

        """
        self.yellow_vision_sender.send(vision)
        self.yellow_primitive_set_sender.send(primitive_set)
