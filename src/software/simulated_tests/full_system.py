from subprocess import Popen

from proto.import_all_protos import *

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.py_constants import *


class FullSystem(object):
    def __init__(self, runtime_dir="/tmp/tbots"):

        """Runs our standalone er-force simulator binary and sets up the unix
        sockets to communicate with it

        :param runtime_dir: The runtime directory

        """

        # inputs to full_system
        self.robot_status_sender = ThreadedUnixSender(runtime_dir + ROBOT_STATUS_PATH)
        self.ssl_wrapper_sender = ThreadedUnixSender(runtime_dir + SSL_WRAPPER_PATH)
        self.ssl_referee_sender = ThreadedUnixSender(runtime_dir + SSL_REFEREE_PATH)
        self.sensor_proto_sender = ThreadedUnixSender(runtime_dir + SENSOR_PROTO_PATH)

        # outputs from full_system
        self.world_listener = ThreadedUnixListener(runtime_dir + WORLD_PATH, World)
        self.primitive_listener = ThreadedUnixListener(
            runtime_dir + PRIMITIVE_PATH, PrimitiveSet
        )

        # override the tactic
        self.tactic_override = ThreadedUnixSender(runtime_dir + TACTIC_OVERRIDE_PATH)
        self.play_override = ThreadedUnixSender(runtime_dir + PLAY_OVERRIDE_PATH)
        self.game_state_override = ThreadedUnixSender(runtime_dir + GAME_STATE_OVERRIDE_PATH)

        # TODO (#2510) rename to full_system
        self.full_system_process = Popen(["software/unix_full_system"])

    def send_robot_status(self, robot_status):
        """Send the robot_status to full_system

        :param robot_status: The RobotStatus to send

        """
        self.robot_status_sender.send(robot_status)

    def send_ssl_wrapper(self, ssl_wrapper_packet):
        """Send the ssl_wrapper_packet to full_system

        :param ssl_wrapper_packet: The packet to send

        """
        self.ssl_wrapper_sender.send(ssl_wrapper_packet)

    def send_ssl_referee(self, ssl_referee_packet):
        """Send the ssl_referee_packet to full_system

        :param ssl_referee_packet: The packet to send

        """
        self.ssl_referee_sender.send(ssl_referee_packet)

    def send_sensor_proto(self, sensor_proto):
        """Send a sensor msg to full system.

        :param sensor_proto: The sensor msg to send

        """
        self.sensor_proto_sender.send(sensor_proto)

    def send_tactic_override(self, assigned_tactic_play_control_params):
        """Send the control params for the assigned tactic play to
        run specific tactics on assigned robots.

        :param assigned_tactic_play_control_params:
                The control params of the AssignedTacticPlay
        
        """
        self.tactic_override.send(assigned_tactic_play_control_params)

    def send_play_override(self, play):
        """Send the play to run
        
        :param play:
                The play to run
        """
        self.play_override.send(play)
    
    #TODO remove as part of issue 2551
    def send_game_state_override(self, game_state):
        """Send the gamestate to override
        
        :param game_state:
                The game state to override
        """
        self.game_state_override.send(game_state)

    def get_world(self, block=False):
        """Grabs the world msg from the buffer if it exists, returns None
        if buffer is empty.

        :param block: Whether or not we should block until we receive a packet
        :return: World or None

        """
        return self.world_listener.get_most_recent_message(block)

    def get_primitive_set(self):
        """Grabs the primitive msg set from the buffer if it exists, returns
        None if buffer is empty.

        :return: PrimitiveSet or None

        """
        return self.primitive_listener.get_most_recent_message()

    def stop():
        """Stop all listeners and senders.

        """

        for unix_socket in [
            self.robot_status_sender,
            self.ssl_wrapper_sender,
            self.ssl_referee_sender,
            self.tactic_override,
            self.sensor_proto_sender,
            self.world_listener,
        ]:
            unix_socket.force_stop()
        self.primitive_listener.force_stop()
