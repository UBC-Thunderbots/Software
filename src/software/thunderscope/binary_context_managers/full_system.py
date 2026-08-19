from __future__ import annotations

import logging
import os
import subprocess
import threading
import time
from subprocess import Popen, TimeoutExpired
import re

from software.py_constants import *
from software.python_bindings import *

from proto.import_all_protos import *
from software.py_constants import *
from software.thunderscope.constants import LogLevels
from software.thunderscope.binary_context_managers.util import *
from software.thunderscope.gl.layers.gl_obstacle_layer import ObstacleList
from software.thunderscope.proto_unix_io import ProtoUnixIO


class FullSystem:
    """Full System Binary Context Manager"""

    def __init__(
        self,
        path_to_binary: str,
        full_system_runtime_dir: os.PathLike = None,
        debug_full_system: bool = False,
        friendly_colour_yellow: bool = False,
        should_restart_on_crash: bool = True,
        run_sudo: bool = False,
        running_in_realtime: bool = True,
        log_level: LogLevels = LogLevels.DEBUG,
    ) -> None:
        """Run FullSystem

        :param path_to_binary: The path of the binary used for this unix full system
        :param full_system_runtime_dir: The directory to run the blue full_system in
        :param debug_full_system: Whether to run the full_system in debug mode
        :param friendly_color_yellow: a argument passed into the unix_full_system binary (--friendly_colour_yellow)
        :param should_restart_on_crash: whether or not to restart the program after it has been crashed
        :param run_sudo: true if we should run full system under sudo
        :param running_in_realtime: True if we are running fullsystem in realtime, else False
        :param log_level: Minimum g3log level that will be printed (DEBUG|INFO|WARNING|FATAL)
        """
        self.path_to_binary = path_to_binary
        self.full_system_runtime_dir = full_system_runtime_dir
        self.generic_command = [
            # We keep the path relative to match processes that might have been
            # started in different working directories, but keep the runtime dir
            # the same as this one so we don't kill other fullsystems
            self.path_to_binary,
            "--runtime_dir={}".format(self.full_system_runtime_dir),
        ]
        self.debug_full_system = debug_full_system
        self.friendly_colour_yellow = friendly_colour_yellow
        self.full_system_proc = None
        self.should_restart_on_crash = should_restart_on_crash
        self.should_run_under_sudo = run_sudo
        self.running_in_realtime = running_in_realtime
        self.log_level = log_level
        self.thread = threading.Thread(target=self.__restart__, daemon=True)

    def discover_supported_flags(self, path_to_binary: str) -> set[str]:
        """Discover what binary flags are supported by provided binary

        :param path_to_binary path to specific binary
        :return a set of supported flags
        """
        try:
            result = subprocess.run(
                [path_to_binary, "--help"], capture_output=True, text=True, timeout=3
            )
            flags = re.findall(r"--(\w+)", result.stdout)
            return set(flags)
        except (subprocess.TimeoutExpired, FileNotFoundError, PermissionError):
            return set()

    def __enter__(self) -> FullSystem:
        """Enter the full_system context manager.

        If the debug mode is enabled then the binary is _not_ run and the
        command to debug under gdb is printed. The  context manager will then
        wait for the binary to be launched before continuing.

        :return: full_system context managed instance

        """
        # Setup unix socket directory
        try:
            os.makedirs(self.full_system_runtime_dir)
        except:
            pass

        supported_flags = self.discover_supported_flags(self.path_to_binary)

        cmd_parts = [self.path_to_binary]
        # runtime_dir is always required (core functionality)
        cmd_parts.append("--runtime_dir={}".format(self.full_system_runtime_dir))

        # Optional flags - only add if supported
        if self.friendly_colour_yellow and "friendly_colour_yellow" in supported_flags:
            cmd_parts.append("--friendly_colour_yellow")
        if not self.running_in_realtime and "ci" in supported_flags:
            cmd_parts.append("--ci")
        if "log_level" in supported_flags:
            cmd_parts.append("--log_level={}".format(self.log_level.value))

        # Log supported flags info based on importance level
        if supported_flags:
            logging.debug("Binary support flags: {}".format(supported_flags))
        else:
            logging.warning(
                "Could not discover flags for path: '{}'. Continuing...".format(
                    self.path_to_binary
                )
            )

        self.full_system = " ".join(cmd_parts)

        if self.should_run_under_sudo:
            if not is_cmd_running(self.generic_command):
                logging.info(
                    (
                        f"""
Run Fullsystem under sudo ==============
1. Build the full system:

./tbots.py build unix_full_system

2. Run the following binaries from src to run under sudo:

sudo bazel-bin/{self.full_system}

3. Rerun this binary once the fullsystem is running
"""
                    )
                )
            else:
                # Wait for the user to exit and restart the binary
                while True:
                    time.sleep(1)

        elif self.debug_full_system:
            # We don't want to check the exact command because this binary could
            # be debugged from clion or somewhere other than gdb
            if not is_cmd_running(self.generic_command):
                logging.info(
                    (
                        f"""

Debugging Fullsystem ==============
1. Build the full system in debug mode:

./tbots.py -d build unix_full_system

2. Run the following binaries from src to debug full system:

gdb --args bazel-bin/{self.full_system}

3. Rerun this binary once the gdb instance is setup
"""
                    )
                )

                # Wait for the user to exit and restart the binary
                while True:
                    time.sleep(1)

        else:
            kill_cmd_if_running(self.generic_command)
            self.full_system_proc = Popen(self.full_system.split(" "))
            if self.should_restart_on_crash:
                self.thread.start()

        return self

    def __restart__(self) -> None:
        """Restarts full system."""
        while self.should_restart_on_crash:
            if not is_cmd_running(self.generic_command):
                self.full_system_proc = Popen(self.full_system.split(" "))
                logging.info("FullSystem has restarted.")

            time.sleep(1)

    def __exit__(self, type, value, traceback) -> None:
        """Exit the full_system context manager.

        :param type: The type of exception that was raised
        :param value: The exception that was raised
        :param traceback: The traceback of the exception
        """
        self.should_restart_on_crash = False

        if self.full_system_proc:
            # It's important to terminate full system instead of killing
            # it to allow it to clean up its resources.
            self.full_system_proc.terminate()

            # Kill the process if it doesn't exit in the given time plus some buffer.
            try:
                self.full_system_proc.wait(
                    timeout=MAX_TIME_TO_EXIT_FULL_SYSTEM_SEC + 0.1
                )
            except TimeoutExpired:
                self.full_system_proc.kill()

    def setup_proto_unix_io(self, proto_unix_io: ProtoUnixIO) -> None:
        """Helper to run full system and attach the appropriate unix senders/listeners

        :param proto_unix_io: The unix io to setup for this full_system instance
        """
        # Setup LOG(VISUALIZE) handling from full system. We set from_log_visualize
        # to true to decode from base64.
        for proto_class in [
            PathVisualization,
            PassVisualization,
            AttackerVisualization,
            CostVisualization,
            NamedValue,
            PlayInfo,
            ObstacleList,
            DebugShapes,
            BallPlacementVisualization,
        ]:
            proto_unix_io.attach_unix_receiver(
                runtime_dir=self.full_system_runtime_dir,
                proto_class=proto_class,
                from_log_visualize=True,
            )

        proto_unix_io.attach_unix_receiver(
            self.full_system_runtime_dir, "/log", RobotLog
        )

        # Outputs from full_system
        proto_unix_io.attach_unix_receiver(
            self.full_system_runtime_dir, WORLD_PATH, World
        )
        proto_unix_io.attach_unix_receiver(
            self.full_system_runtime_dir, PRIMITIVE_PATH, PrimitiveSet
        )

        # Inputs to full_system
        for arg in [
            (ROBOT_STATUS_PATH, RobotStatus),
            (SSL_WRAPPER_PATH, SSL_WrapperPacket),
            (SSL_REFEREE_PATH, Referee),
            (SENSOR_PROTO_PATH, SensorProto),
            (TACTIC_OVERRIDE_PATH, AssignedTacticPlayControlParams),
            (PLAY_OVERRIDE_PATH, Play),
            (DYNAMIC_PARAMETER_UPDATE_REQUEST_PATH, ThunderbotsConfig),
            (VALIDATION_PROTO_SET_PATH, ValidationProtoSet),
            (ROBOT_LOG_PATH, RobotLog),
            (ROBOT_CRASH_PATH, RobotCrash),
            (VIRTUAL_OBSTACLES_UNIX_PATH, VirtualObstacles),
            (REPLAY_BOOKMARK_PATH, ReplayBookmark),
        ]:
            proto_unix_io.attach_unix_sender(self.full_system_runtime_dir, *arg)
