from subprocess import Popen

from proto.import_all_protos import *

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.py_constants import *


class ErForceSimulator(object):
    def __init__(self, runtime_dir="/tmp/tbots"):

        """Runs our standalone er-force simulator binary and sets up the unix
        sockets to communicate with it

        :param runtime_dir: The unix path to run everything

        """
        # inputs to er_force_simulator_main
        self.sim_tick_sender = ThreadedUnixSender(runtime_dir + SIMULATION_TICK_PATH)
        self.world_state_sender = ThreadedUnixSender(runtime_dir + WORLD_STATE_PATH)
        self.blue_world_sender = ThreadedUnixSender(runtime_dir + BLUE_WORLD_PATH)
        self.yellow_world_sender = ThreadedUnixSender(runtime_dir + YELLOW_WORLD_PATH)
        self.blue_primitive_set_sender = ThreadedUnixSender(
            runtime_dir + BLUE_PRIMITIVE_SET
        )
        self.yellow_primitive_set_sender = ThreadedUnixSender(
            runtime_dir + YELLOW_PRIMITIVE_SET
        )

        # outputs from er_force_sim_main
        self.ssl_wrapper_listener = ThreadedUnixListener(
            runtime_dir + SSL_WRAPPER_PACKET_PATH, SSL_WrapperPacket
        )
        self.blue_robot_status_listener = ThreadedUnixListener(
            runtime_dir + BLUE_ROBOT_STATUS_PATH, RobotStatus
        )
        self.yellow_robot_status_listener = ThreadedUnixListener(
            runtime_dir + YELLOW_ROBOT_STATUS_PATH, RobotStatus,
        )

        self.world_state = WorldState()

        self.simulator_process = Popen(["software/er_force_simulator_main"])

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
                        x_meters=robot_location.x(), y_meters=robot_location.y()
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

    def setup_ball(self, ball_position, ball_velocity, distance_from_ground=0):
        """Setup the ball with the x, y coordinates in meters

        :param ball_position: A tuple with the x,y coordinates
        :param ball_velocity: A tuple with the x,y velocity components
        :param distance_from_ground: How high up to start the ball

        """
        self.world_state.ball_state.CopyFrom(
            BallState(
                global_position=Point(
                    x_meters=ball_position.x(), y_meters=ball_position.y(),
                ),
                global_velocity=Vector(
                    x_component_meters=ball_velocity.x(),
                    y_component_meters=ball_velocity.y(),
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

        if ssl_wrapper:
            sensor_proto.ssl_vision_msg.CopyFrom(ssl_wrapper)

        robot_status = robot_status_listener.get_most_recent_message()

        packets = []
        while robot_status is not None:
            packets.append(robot_status)
            robot_status = robot_status_listener.get_most_recent_message()

        sensor_proto.robot_status_msgs.extend(packets)
        return sensor_proto

    def get_blue_sensor_proto(self, ssl_wrapper):
        """Returns the blue sensor proto

        :param ssl_wrapper: The wrapper to pack in the sensor proto

        """
        return self.__get_sensor_proto(ssl_wrapper, self.blue_robot_status_listener)

    def get_yellow_sensor_proto(self, ssl_wrapper):
        """Returns the yellow sensor proto

        :param ssl_wrapper: The wrapper to pack in the sensor proto

        """
        return self.__get_sensor_proto(ssl_wrapper, self.yellow_robot_status_listener)

    def get_ssl_wrapper_packet(self, block=False):
        """Get wrapper packet

        :param block: If true, block until we receive a packet
        :return: SSL_WrapperPacket

        """
        return self.ssl_wrapper_listener.get_most_recent_message(block)

    def tick(self, duration_ms):
        """Tick the simulator with the given duration

        :param duration_ms: The duration to step the sim

        """
        tick = SimulatorTick()
        tick.milliseconds = duration_ms
        self.sim_tick_sender.send(tick)

    def send_blue_primitive_set_and_world(self, world, primitive_set):
        """Blue primitive set and world

        :param world: The world msg to send
        :param primitive_set: The primitive set to send

        """
        self.blue_world_sender.send(world)
        self.blue_primitive_set_sender.send(primitive_set)

    def send_yellow_primitive_set_and_world(self, world, primitive_set):
        """Yellow primitive set and world

        :param world: The world msg to send
        :param primitive_set: The primitive set to send

        """
        self.yellow_world_sender.send(world)
        self.yellow_primitive_set_sender.send(primitive_set)

    def stop():
        """Stop all listeners and senders.
        """
        for unix_socket in [
            self.sim_tick_sender,
            self.world_state_sender,
            self.blue_world_sender,
            self.yellow_world_sender,
            self.blue_primitive_set_sender,
            self.yellow_primitive_set_sender,
            self.ssl_wrapper_listener,
            self.blue_robot_status_listener,
            self.yellow_robot_status_listener,
        ]:
            unix_socket.force_stop()
