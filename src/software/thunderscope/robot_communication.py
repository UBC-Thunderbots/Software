from software.py_constants import *
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.python_bindings import *
from proto.import_all_protos import *
import threading
import sys
import time


class RobotCommunication(object):

    """ Communicate with the robots """

    def __init__(
        self,
        full_system_proto_unix_io,
        multicast_channel,
        interface,
        estop_path="/dev/ttyACM0",
        estop_buadrate=115200,
    ):
        """Initialize the communication with the robots

        :param full_system_proto_unix_io: full_system_proto_unix_io object
        :param multicast_channel: The multicast channel to use
        :param interface: The interface to use
        :param estop_path: The path to the estop
        :param estop_baudrate: The baudrate of the estop

        """
        self.full_system_proto_unix_io = full_system_proto_unix_io
        self.multicast_channel = str(multicast_channel)
        self.interface = interface
        self.estop_path = estop_path
        self.estop_buadrate = estop_buadrate

        self.world_buffer = ThreadSafeBuffer(1, World)
        self.primitive_buffer = ThreadSafeBuffer(1, PrimitiveSet)

        self.full_system_proto_unix_io.register_observer(World, self.world_buffer)
        self.full_system_proto_unix_io.register_observer(
            PrimitiveSet, self.primitive_buffer
        )

        try:
            self.estop_reader = ThreadedEstopReader(
                self.estop_path, self.estop_buadrate
            )
        except Exception:
            raise Exception("Could not find estop, make sure its plugged in")

    def run(self):
        """Forward World and PrimitiveSet protos from fullsystem to the robots.

        If the emergency stop is tripped, the PrimitiveSet will not be sent so
        that the robots timeout and stop.

        NOTE: If disconnect_fullsystem_from_robots is called, then the packets
        will not be forwarded to the robots. 

        send_override_primitive_set can be used to send a primitive set, which
        is useful to dip in and out of robot diagnostics.

        """

        # motor_control = MotorControl(MotorControl.drive_control=MotorControl.DirectPerWheelControl(), dribbler_speed_rpm=0.0)
        primitive0 = Primitive(
            direct_control=DirectControlPrimitive(
                motor_control=MotorControl(
                    direct_per_wheel_control=MotorControl.DirectPerWheelControl(
                        front_left_wheel_rpm=0
                    ),
                    dribbler_speed_rpm=0,
                ),
                power_control=PowerControl(
                    charge_mode=PowerControl.ChargeMode.CHARGE,
                    chicker=PowerControl.ChickerControl(chip_distance_meters=3),
                ),
            )
        )
        print("SENDING")
        set = [{3: primitive0}]
        primitiveset = PrimitiveSet(
            time_sent=Timestamp(), stay_away_from_ball=False, robot_primitives=set[0]
        )
        self.send_primitive_mcast_sender.send_proto(primitiveset)
        time.sleep(1)
        sys.exit(1)
        # primitive1 = Primitive(
        # direct_control=DirectControlPrimitive(
        # motor_control=MotorControl(
        # direct_velocity_control=MotorControl.DirectVelocityControl(
        # velocity=Vector(
        # x_component_meters=0.0, y_component_meters=speed
        # ),
        # angular_velocity=AngularVelocity(radians_per_second=0.0),
        # ),
        # dribbler_speed_rpm=-10000,
        # )
        # )
        # )
        # primitive2 = Primitive(
        # direct_control=DirectControlPrimitive(
        # motor_control=MotorControl(
        # direct_velocity_control=MotorControl.DirectVelocityControl(
        # velocity=Vector(
        # x_component_meters=0.0, y_component_meters=-speed
        # ),
        # angular_velocity=AngularVelocity(radians_per_second=0.0),
        # ),
        # dribbler_speed_rpm=-10000,
        # )
        # )
        # )
        # primitive3 = Primitive(
        # direct_control=DirectControlPrimitive(
        # motor_control=MotorControl(
        # direct_velocity_control=MotorControl.DirectVelocityControl(
        # velocity=Vector(
        # x_component_meters=speed, y_component_meters=speed
        # ),
        # angular_velocity=AngularVelocity(radians_per_second=0.0),
        # ),
        # dribbler_speed_rpm=-10000,
        # )
        # )
        # )
        # primitive4 = Primitive(
        # direct_control=DirectControlPrimitive(
        # motor_control=MotorControl(
        # direct_velocity_control=MotorControl.DirectVelocityControl(
        # velocity=Vector(
        # x_component_meters=-speed, y_component_meters=-speed
        # ),
        # angular_velocity=AngularVelocity(radians_per_second=0.0),
        # ),
        # dribbler_speed_rpm=-10000,
        # )
        # )
        # )

        # primitive0_dribbler = Primitive(
        # direct_control=DirectControlPrimitive(
        # motor_control=MotorControl(
        # direct_per_wheel_control=MotorControl.DirectPerWheelControl(
        # front_left_wheel_rpm=0
        # ),
        # dribbler_speed_rpm=-10000,
        # )
        # )
        # )

        # set = [{3: primitive1}, {3: primitive2}, {3: primitive3}, {3: primitive4}]
        # # primitiveset = PrimitiveSet(time_sent = Timestamp(),stay_away_from_ball=False, robot_primitives=set)

        # while True:
        # if self.fullsystem_connected_to_robots:

        # # Send the world
        # # world = self.world_buffer.get(block=True)
        # # self.send_world.send_proto(world)

        # # Send the primitive set
        # # primitive_set = self.primitive_buffer.get(block=False)

        # for i in range(4):
        # primitiveset = PrimitiveSet(
        # time_sent=Timestamp(),
        # stay_away_from_ball=False,
        # robot_primitives=set[i],
        # )
        # primitive_set = primitiveset
        # self.send_primitive_mcast_sender.send_proto(primitive_set)
        # logging.info(f"running index i {i}")
        # # if self.estop_reader.isEstopPlay():
        # #     self.send_primitive_mcast_sender.send_proto(primitive_set)

        # time.sleep(2)
        # primitiveset_stop = PrimitiveSet(
        # time_sent=Timestamp(),
        # stay_away_from_ball=False,
        # robot_primitives={3: primitive0_dribbler},
        # )
        # self.send_primitive_mcast_sender.send_proto(primitiveset_stop)
        # time.sleep(2)

        # primitiveset_stop = PrimitiveSet(
        # time_sent=Timestamp(),
        # stay_away_from_ball=False,
        # robot_primitives={3: primitive0},
        # )
        # self.send_primitive_mcast_sender.send_proto(primitiveset_stop)

        # exit(0)

    def connect_fullsystem_to_robots(self):
        """ Connect the robots to fullsystem """

        self.fullsystem_connected_to_robots = True

    def disconnect_fullsystem_from_robots(self):
        """ Disconnect the robots from fullsystem """

        self.fullsystem_connected_to_robots = False

    def send_override_primitive_set(self, primitive_set):
        """Send an override primitive set to the robots

        NOTE: We will only send the overridden proto if the estop is enabled.

        """
        if self.robots_connected_to_fullsystem:
            raise Exception(
                "Currently connected to fullsystem" + " can not override primitive set"
            )

        if self.estop_reader.isEstopPlay():
            self.primitive_set_mcast_sender.send(primitive_set)

    def __enter__(self):
        """Enter RobotCommunication context manager. Setup multicast listener
        for RobotStatus and multicast senders for World and PrimitiveSet

        """
        # Create the multicast channels
        # self.receive_robot_status = RobotStatusProtoListener(
        #     self.multicast_channel + "%" + self.interface,
        #     ROBOT_STATUS_PORT,
        #     lambda data: print(data),
        #     True,
        # )

        self.send_primitive_mcast_sender = PrimitiveSetProtoSender(
            self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
        )

        self.receive_robot_log = RobotLogProtoListener(
            self.multicast_channel + "%" + self.interface,
            ROBOT_LOGS_PORT,
            lambda data: self.full_system_proto_unix_io.send_proto(RobotLog, data),
            True,
        )

        self.receive_robot_log = SSLWrapperPacketProtoListener(
            SSL_ADDRESS,
            SSL_PORT,
            lambda data: self.full_system_proto_unix_io.send_proto(
                SSL_WrapperPacket, data
            ),
            True,
        )

        self.send_primitive_set = PrimitiveSetProtoSender(
            self.multicast_channel + "%" + self.interface, PRIMITIVE_PORT, True
        )

        self.send_world = WorldProtoSender(
            self.multicast_channel + "%" + self.interface, VISION_PORT, True
        )

        self.connect_fullsystem_to_robots()
        self.run_thread = threading.Thread(target=self.run)
        self.run_thread.start()

    def __exit__(self):
        """Exit RobotCommunication context manager

        """
        self.run_thread.join()
