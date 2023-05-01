from software.py_constants import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.common.common_widgets import IndividualRobotMode
from software.python_bindings import *
from proto.import_all_protos import *
from pyqtgraph.Qt import QtCore
import threading
import time
import pdb
import traceback


class RobotCommunication(object):

    """ Communicate with the robots """

    def __init__(
        self,
        current_proto_unix_io,
        multicast_channel,
        interface,
        disable_estop,
        estop_path="/dev/ttyACM0",
        estop_buadrate=115200,
    ):
        # pdb.set_trace()
        """Initialize the communication with the robots

        :param current_proto_unix_io: the current proto unix io object
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use
        :param disable_estop: whether to disable estop checks (ONLY FOR TESTING LOCALLY)
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop

        """
        self.sequence_number = 0
        self.last_time = time.time()
        self.current_proto_unix_io = current_proto_unix_io
        self.multicast_channel = str(multicast_channel)
        self.interface = interface
        self.disable_estop = disable_estop
        self.estop_path = estop_path
        self.estop_buadrate = estop_buadrate

        self.world_buffer = ThreadSafeBuffer(1, World,owner="RobotCommunication.world_buffer")
        self.primitive_buffer = ThreadSafeBuffer(1, PrimitiveSet,owner="RobotCommunication.primitive_buffer")

        self.motor_control_diagnostics_buffer = ThreadSafeBuffer(1, MotorControl,owner="RobotCommunication.motor_control_diagnostics")
        self.power_control_diagnostics_buffer = ThreadSafeBuffer(1, PowerControl,owner="RobotCommunication.power_control_diagnostics")

        self.current_proto_unix_io.register_observer(World, self.world_buffer)

        self.robots_connected_to_fullsystem = set()
        self.robots_connected_to_manual = set()
        self.robots_to_be_disconnected = {}

        self.current_proto_unix_io.register_observer(
            PrimitiveSet, self.primitive_buffer
        )

        self.current_proto_unix_io.register_observer(
            MotorControl, self.motor_control_diagnostics_buffer
        )
        self.current_proto_unix_io.register_observer(
            PowerControl, self.power_control_diagnostics_buffer
        )

        self.send_estop_state_thread = threading.Thread(target=self.__send_estop_state)
        self.run_thread = threading.Thread(target=self.run,daemon=True)
        self.stop_running = False

        # only checks for estop if checking is not disabled
        if not self.disable_estop:
            try:
                self.estop_reader = ThreadedEstopReader(
                    self.estop_path, self.estop_buadrate
                )
            except Exception:
                raise Exception("Could not find estop, make sure its plugged in")

    def __send_estop_state(self):
        if not self.disable_estop:
            while not self.stop_running:
                self.current_proto_unix_io.send_proto(
                    EstopState, EstopState(is_playing=self.estop_reader.isEstopPlay())
                )
                time.sleep(0.1)

    def run(self):
        """Forward World and PrimitiveSet protos from fullsystem to the robots.

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.

        NOTE: If disconnect_fullsystem_from_robots is called, then the packets
        will not be forwarded to the robots.

        send_override_primitive_set can be used to send a primitive set, which
        is useful to dip in and out of robot diagnostics.

        """
        while not self.stop_running:
            # total primitives for all robots
            robot_primitives = {}

            # fullsystem is running, so world data is being received
            if self.robots_connected_to_fullsystem:
                # print("robot_communication.py line 108: world_buffer.get",flush=True)
                # print(traceback.print_stack())
                world = self.world_buffer.get(block=True)

                # send the world proto
                #try:
                #if world is not None:
                self.send_world.send_proto(world)
                #except:
                #    print("proto error")

                # Get the primitives
                # print("robot_communication.py line 115: world_buffer.get",flush=True)
                primitive_set = self.primitive_buffer.get(block=False)

                robot_primitives = dict(primitive_set.robot_primitives)

            # get the manual control primitive
            # print("robot_communication.py line 122: diagnostics.get",flush=True)
            diagnostics_primitive = DirectControlPrimitive(
                motor_control=self.motor_control_diagnostics_buffer.get(block=False),
                power_control=self.power_control_diagnostics_buffer.get(block=False),
            )

            # diagnostics is running
            if self.robots_connected_to_manual:
                # for all robots connected to diagnostics, set their primitive
                for robot_id in self.robots_connected_to_manual:
                    robot_primitives[robot_id] = Primitive(
                        direct_control=diagnostics_primitive
                    )

            # sends a final stop primitive to all disconnected robots and removes them from list
            # in order to prevent robots acting on cached old primitives
            for robot_id, num_times_to_stop in self.robots_to_be_disconnected.items():
                if num_times_to_stop > 0:
                    robot_primitives[robot_id] = Primitive(stop=StopPrimitive())
                    self.robots_to_be_disconnected[robot_id] = num_times_to_stop - 1

            # initialize total primitive set and send it
            primitive_set = PrimitiveSet(
                time_sent=Timestamp(epoch_timestamp_seconds=time.time()),
                stay_away_from_ball=False,
                robot_primitives=robot_primitives,
                sequence_number=self.sequence_number,
            )

            self.sequence_number += 1

            if not self.disable_estop and self.estop_reader.isEstopPlay():
                self.last_time = primitive_set.time_sent.epoch_timestamp_seconds
                self.send_primitive_set.send_proto(primitive_set)

            time.sleep(0.01)

    def toggle_robot_connection(self, mode, robot_id):
        """
        Connects a robot to or disconnects a robot from diagnostics

        :param mode: the mode of input for this robot's primitives
        :param robot_id: the id of the robot to be added or removed from the diagnostics set
        """
        self.robots_connected_to_fullsystem.discard(robot_id)
        self.robots_connected_to_manual.discard(robot_id)
        self.robots_to_be_disconnected.pop(robot_id, None)

        if mode == IndividualRobotMode.NONE:
            self.robots_to_be_disconnected[robot_id] = NUM_TIMES_SEND_STOP
        elif mode == IndividualRobotMode.MANUAL:
            self.robots_connected_to_manual.add(robot_id)
        elif mode == IndividualRobotMode.AI:
            self.robots_connected_to_fullsystem.add(robot_id)

    def setup_for_fullsystem(self):
        """
        Sets up a world sender, a listener for SSL vision data, and connects all robots to fullsystem as default
        """
        print("M",flush=True)
        def test_func(data):
            # print(data,flush=True)
            if not self.stop_running:
                self.current_proto_unix_io.send_proto(SSL_WrapperPacket, data)

        self.receive_ssl_wrapper = SSLWrapperPacketProtoListener(
            SSL_VISION_ADDRESS,
            SSL_VISION_PORT,
            # lambda data: self.current_proto_unix_io.send_proto(SSL_WrapperPacket, data),
            test_func,
            True,
        )

        self.send_world = WorldProtoSender(
            self.multicast_channel + "%" + self.interface, VISION_PORT, True
        )

        self.robots_connected_to_fullsystem = {
            robot_id for robot_id in range(MAX_ROBOT_IDS_PER_SIDE)
        }

    def __enter__(self):
        """Enter RobotCommunication context manager. Setup multicast listener
        for RobotStatus and multicast senders for World and PrimitiveSet

        """
        # Create the multicast listeners
        def __forward_to_proto_unix_io__(data, type):
            if not self.stop_running:
                self.current_proto_unix_io.send_proto(type, data)

        self.receive_robot_status = RobotStatusProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_STATUS_PORT,
            #lambda data: if not self.stop_running: self.current_proto_unix_io.send_proto(RobotStatus, data),
            lambda data: __forward_to_proto_unix_io__(data, RobotStatus),
            True,
        )

        self.receive_robot_log = RobotLogProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_LOGS_PORT,
            #lambda data: self.current_proto_unix_io.send_proto(RobotLog, data),
            lambda data: __forward_to_proto_unix_io__(data, RobotLog),
            True,
        )

        # Create multicast senders
        self.send_primitive_set = PrimitiveSetProtoSender(
            self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
        )

        self.send_estop_state_thread.start()
        self.run_thread.start()

        return self

    def __exit__(self,type,value,traceback):
        """Exit RobotCommunication context manager

        """
        self.stop_running = True
        self.current_proto_unix_io.force_close()
        self.current_proto_unix_io.send_proto(SSL_WrapperPacket,None)
        self.current_proto_unix_io = None
        time.sleep(2)
        print("killing robocom",flush=True)
        self.run_thread.join(10)
        print("run_thread joined",flush=True)
