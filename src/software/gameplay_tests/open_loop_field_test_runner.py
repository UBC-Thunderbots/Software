# util must be imported first, since it preloads the Qt libs before any Qt
# binding is imported
# ruff: noqa: E402
from software.gameplay_tests.util import load_command_line_arguments

import math
import queue
import threading
import time

import proto.import_all_protos as protos
import pytest
import software.python_bindings as tbots_cpp
from software.gameplay_tests.plotjuggler_sender import PlotJugglerSender
from software.logger.logger import create_logger
from software.py_constants import (
    MAX_ROBOT_IDS_PER_SIDE,
    SSL_VISION_ADDRESS,
    SSL_VISION_PORT,
    getRobotMulticastChannel,
)
from software.thunderscope.constants import EstopMode, IndividualRobotMode
from software.thunderscope.estop_helpers import get_estop_config
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.robot_communication import RobotCommunication
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.thunderscope_config import (
    configure_open_loop_field_test_view,
)
from software.thunderscope.wifi_communication_manager import WifiCommunicationManager

logger = create_logger(__name__)

ROBOT_STATUS_SURVEY_TIMEOUT_S = 5.0
ROBOT_STATUS_BUFFER_SIZE = 1000
# Time to let RobotCommunication flush the queued stop primitives before teardown
STOP_FLUSH_DELAY_S = 0.5
MOTOR_CONTROL_RESEND_PERIOD_S = 0.05
SSL_WRAPPER_BUFFER_SIZE = 5
VISION_POSE_TIMEOUT_S = 2.0


class OpenLoopFieldTestRunner:
    """Run a field test that drives robots open-loop, without FullSystem or vision.

    Robots are put into MANUAL mode and commanded with MotorControl protos, the
    same path Thunderscope's Robot Diagnostics uses. Commanded velocities are in
    the robot's local frame. Since there is no vision, the only feedback is the
    robot's own RobotStatus (e.g. its estimated local velocity), so results have
    to be validated by eye or from the recorded statuses.
    """

    def __init__(
        self,
        thunderscope,
        rerun_test_widget,
        proto_unix_io,
        robot_communication,
        friendly_colour_yellow=False,
        interface=None,
        vision_robot_id=None,
    ):
        """Initialize the OpenLoopFieldTestRunner

        :param thunderscope: The Thunderscope to use
        :param rerun_test_widget: The RerunTestWidget used to rerun the test
        :param proto_unix_io: The proto unix io the robots communicate over
        :param robot_communication: The robot communication instance
        :param friendly_colour_yellow: True if the friendly team is yellow, else blue
        :param interface: The network interface used to send data to PlotJuggler
        :param vision_robot_id: The id vision sees the tracked robot as, if it
            differs from the robot's own id. None to use the robot's id
        """
        self.thunderscope = thunderscope
        self.friendly_colour_yellow = friendly_colour_yellow
        self.vision_robot_id = vision_robot_id
        self.plotjuggler_sender = PlotJugglerSender(interface)
        self.ssl_wrapper_buffer = ThreadSafeBuffer(
            SSL_WRAPPER_BUFFER_SIZE, protos.SSL_WrapperPacket
        )
        proto_unix_io.register_observer(
            protos.SSL_WrapperPacket, self.ssl_wrapper_buffer
        )
        self.tracked_robot_id = None
        self.initial_pose = None
        self.expected_position = (0.0, 0.0)
        self.rerun_test_widget = rerun_test_widget
        self.proto_unix_io = proto_unix_io
        self.robot_communication = robot_communication
        self.last_exception = None

        self.test_fn = None
        self.test_thread = None
        self.rerun_test_widget.rerun_requested.connect(self.__start_test_thread)

        self.robot_status_buffer = ThreadSafeBuffer(
            ROBOT_STATUS_BUFFER_SIZE, protos.RobotStatus
        )
        self.proto_unix_io.register_observer(
            protos.RobotStatus, self.robot_status_buffer
        )

        logger.info("determining connected robots from RobotStatus")
        self.connected_robot_ids = set()
        survey_end_time = time.time() + ROBOT_STATUS_SURVEY_TIMEOUT_S
        while time.time() < survey_end_time:
            try:
                robot_status = self.robot_status_buffer.get(
                    block=True, timeout=0.1, return_cached=False
                )
                self.connected_robot_ids.add(robot_status.robot_id)
            except queue.Empty:
                continue

        if len(self.connected_robot_ids) == 0:
            raise Exception("no robots sent a RobotStatus within timeout")
        logger.info(f"connected robot ids {sorted(self.connected_robot_ids)}")

        self.controlled_robot_ids = set()

    def wait_for_estop_play(self):
        """Blocks until the estop is in the PLAY state"""
        if self.robot_communication.estop_is_playing:
            return

        logger.info("\x1b[33m" + "Waiting for Estop to be in PLAY state..." + "\x1b[0m")
        while not self.robot_communication.estop_is_playing:
            time.sleep(0.1)
        logger.info(
            "\x1b[32m" + "Estop is in PLAY state. Proceeding with test." + "\x1b[0m"
        )

    def get_vision_pose(self, robot_id):
        """Get the latest pose of a friendly robot from SSL vision

        :param robot_id: the robot id to look for
        :return: (x_m, y_m, orientation_rad) or None if vision has not seen the robot
        """
        wrapper = self.ssl_wrapper_buffer.get(block=False, return_cached=True)
        if wrapper is None or not wrapper.HasField("detection"):
            return None
        robots = (
            wrapper.detection.robots_yellow
            if self.friendly_colour_yellow
            else wrapper.detection.robots_blue
        )
        for robot in robots:
            if robot.robot_id == robot_id:
                return robot.x / 1000.0, robot.y / 1000.0, robot.orientation
        return None

    def start_path_tracking(self, robot_id):
        """Record the robot's current vision pose as the start of the expected
        path. From now on drive() sends the vision position and the position the
        robot should be at, given the commanded velocities, to PlotJuggler.

        :param robot_id: the robot to track
        """
        # Vision may see the robot under a different id than it reports itself as
        self.tracked_robot_id = (
            robot_id if self.vision_robot_id is None else self.vision_robot_id
        )
        self.initial_pose = None
        deadline = time.time() + VISION_POSE_TIMEOUT_S
        while self.initial_pose is None and time.time() < deadline:
            self.initial_pose = self.get_vision_pose(self.tracked_robot_id)
            time.sleep(0.05)
        if self.initial_pose is None:
            logger.warning(
                f"no vision pose for robot {self.tracked_robot_id}, not tracking path"
            )
            return
        self.expected_position = self.initial_pose[:2]
        logger.info(f"tracking path from initial pose {self.initial_pose}")

    def __send_path_tracking(self, x_m_per_s, y_m_per_s, dt):
        """Advance the expected position by one step and send it, with the
        current vision position, to PlotJuggler

        :param x_m_per_s: commanded local-frame forward velocity
        :param y_m_per_s: commanded local-frame leftward velocity
        :param dt: seconds the command has been applied for
        """
        if self.initial_pose is None:
            return
        # The square is open-loop and never rotates, so the local-frame command
        # maps to the global frame through the initial orientation
        theta = self.initial_pose[2]
        self.expected_position = (
            self.expected_position[0]
            + (x_m_per_s * math.cos(theta) - y_m_per_s * math.sin(theta)) * dt,
            self.expected_position[1]
            + (x_m_per_s * math.sin(theta) + y_m_per_s * math.cos(theta)) * dt,
        )
        values = {
            "open_loop/expected_x": self.expected_position[0],
            "open_loop/expected_y": self.expected_position[1],
        }
        vision_pose = self.get_vision_pose(self.tracked_robot_id)
        if vision_pose is not None:
            values["open_loop/vision_x"] = vision_pose[0]
            values["open_loop/vision_y"] = vision_pose[1]
            values["open_loop/vision_orientation"] = vision_pose[2]
        self.plotjuggler_sender.send(values)

    def set_controlled_robots(self, robot_ids):
        """Choose which robots receive the open-loop commands. All controlled
        robots receive the same command; every other robot is set to NONE.

        :param robot_ids: iterable of robot ids to control
        """
        self.controlled_robot_ids = set(robot_ids)
        for robot_id in range(MAX_ROBOT_IDS_PER_SIDE):
            self.robot_communication.toggle_individual_robot_control_mode(
                robot_id,
                IndividualRobotMode.MANUAL
                if robot_id in self.controlled_robot_ids
                else IndividualRobotMode.NONE,
            )

    def send_velocity(
        self,
        x_m_per_s=0.0,
        y_m_per_s=0.0,
        angular_velocity_rad_per_s=0.0,
        dribbler_speed_rpm=0,
    ):
        """Command a local-frame velocity to the controlled robots. The command
        is held until another one is sent.

        :param x_m_per_s: forward velocity in the robot's local frame
        :param y_m_per_s: leftward velocity in the robot's local frame
        :param angular_velocity_rad_per_s: angular velocity, CCW positive
        :param dribbler_speed_rpm: dribbler speed
        """
        motor_control = protos.MotorControl(dribbler_speed_rpm=dribbler_speed_rpm)
        motor_control.direct_velocity_control.velocity.x_component_meters = x_m_per_s
        motor_control.direct_velocity_control.velocity.y_component_meters = y_m_per_s
        motor_control.direct_velocity_control.angular_velocity.radians_per_second = (
            angular_velocity_rad_per_s
        )
        self.proto_unix_io.send_proto(protos.MotorControl, motor_control)

    def drive(
        self,
        x_m_per_s=0.0,
        y_m_per_s=0.0,
        angular_velocity_rad_per_s=0.0,
        duration_s=1.0,
        dribbler_speed_rpm=0,
    ):
        """Command a local-frame velocity for a fixed duration, then leave it
        commanded (call stop() or drive() again afterwards).

        :param x_m_per_s: forward velocity in the robot's local frame
        :param y_m_per_s: leftward velocity in the robot's local frame
        :param angular_velocity_rad_per_s: angular velocity, CCW positive
        :param duration_s: how long to command this velocity for
        :param dribbler_speed_rpm: dribbler speed
        :return: RobotStatuses received from controlled robots during the drive
        """
        self.__drain_robot_statuses()
        robot_statuses = []
        # Resend periodically like the diagnostics widget does, since
        # RobotCommunication's MotorControl buffer drops protos when full
        last_time = time.time()
        end_time = last_time + duration_s
        while time.time() < end_time:
            self.send_velocity(
                x_m_per_s, y_m_per_s, angular_velocity_rad_per_s, dribbler_speed_rpm
            )
            time.sleep(MOTOR_CONTROL_RESEND_PERIOD_S)
            now = time.time()
            self.__send_path_tracking(x_m_per_s, y_m_per_s, now - last_time)
            last_time = now
            robot_statuses += self.__drain_robot_statuses()
        return robot_statuses

    def stop(self, settle_time_s=1.0):
        """Command zero velocity and wait for the robots to settle.

        An explicit zero velocity is sent, since a MotorControl without a drive
        control leaves the robot's previous wheel targets in place.

        :param settle_time_s: how long to wait after commanding zero velocity
        """
        self.drive(duration_s=settle_time_s)

    def run(self, test_fn):
        """Run test_fn in a background thread while Thunderscope runs on the
        main thread. Blocks until the Thunderscope window is closed. The
        controlled robots are stopped once test_fn finishes or fails, and it
        can be rerun from the Rerun Test button.

        The test fails if any run of test_fn failed.

        :param test_fn: function that takes no arguments and runs the test
        """
        self.test_fn = test_fn
        self.__start_test_thread()
        self.thunderscope.show()
        self.test_thread.join()

        if self.last_exception:
            pytest.fail(str(self.last_exception))

    def __start_test_thread(self):
        """Run test_fn in a new background thread, unless a run is in progress"""
        if self.test_thread is not None and self.test_thread.is_alive():
            logger.warning("Test is already running, not rerunning it")
            return

        def __runner():
            try:
                self.test_fn()
            except Exception as e:
                self.last_exception = e
                raise
            finally:
                self.send_velocity()
                self.set_controlled_robots([])
                self.rerun_test_widget.test_running = False
                logger.info(
                    "Test finished, press Rerun Test to run it again "
                    "or close Thunderscope to exit"
                )

        self.rerun_test_widget.test_running = True
        self.test_thread = threading.Thread(target=__runner, daemon=True)
        self.test_thread.start()

    def __drain_robot_statuses(self):
        """Empty the RobotStatus buffer

        :return: the RobotStatuses from controlled robots that were in the buffer
        """
        robot_statuses = []
        while True:
            robot_status = self.robot_status_buffer.get(
                block=False, return_cached=False
            )
            if robot_status is None:
                return robot_statuses
            if robot_status.robot_id in self.controlled_robot_ids:
                robot_statuses.append(robot_status)


@pytest.fixture
def open_loop_field_test_runner():
    """Runs an open-loop field test: no FullSystem, vision or gamecontroller.
    Robots are commanded directly over WiFi, gated by the estop.

    :return: yields the runner to the test fixture
    """
    args = load_command_line_arguments()
    proto_unix_io = ProtoUnixIO()

    estop_mode, estop_path = get_estop_config(
        args.keyboard_estop, args.disable_communication
    )

    with (
        WifiCommunicationManager(
            current_proto_unix_io=proto_unix_io,
            multicast_channel=getRobotMulticastChannel(args.channel),
            should_setup_full_system=False,
            interface=args.interface,
        ) as wifi_communication_manager,
        RobotCommunication(
            current_proto_unix_io=proto_unix_io,
            communication_manager=wifi_communication_manager,
            estop_mode=estop_mode,
            estop_path=estop_path,
            estop_baudrate=args.estop_baudrate,
        ) as robot_communication,
    ):
        # Only vision is needed for plotting the tracked path, not FullSystem
        ssl_wrapper_listener = None
        vision_interface = args.vision_interface or args.interface
        if vision_interface:
            ssl_wrapper_listener = tbots_cpp.SSLWrapperPacketProtoListener(
                SSL_VISION_ADDRESS,
                SSL_VISION_PORT,
                vision_interface,
                lambda data: proto_unix_io.send_proto(protos.SSL_WrapperPacket, data),
                True,
            )
        else:
            logger.warning(
                "no --vision_interface or --interface given, not listening to SSL vision"
            )

        tscope_config, rerun_test_widget = configure_open_loop_field_test_view(
            proto_unix_io
        )
        tscope = Thunderscope(tscope_config, layout_path=None)

        if estop_mode == EstopMode.KEYBOARD_ESTOP:
            tscope.keyboard_estop_shortcut.activated.connect(
                robot_communication.toggle_keyboard_estop
            )

        runner = OpenLoopFieldTestRunner(
            thunderscope=tscope,
            rerun_test_widget=rerun_test_widget,
            proto_unix_io=proto_unix_io,
            robot_communication=robot_communication,
            friendly_colour_yellow=args.run_yellow,
            interface=args.interface,
            vision_robot_id=args.vision_robot_id,
        )

        yield runner

        # Make sure no robot is left driving on its last command
        runner.set_controlled_robots([])
        time.sleep(STOP_FLUSH_DELAY_S)
