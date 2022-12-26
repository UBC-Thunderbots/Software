import os
import time
import threading
import argparse
import numpy

from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.binary_context_managers import *
from proto.message_translation import tbots_protobuf
import software.python_bindings as cpp_bindings
from software.py_constants import *
from software.thunderscope.robot_communication import RobotCommunication
from software.thunderscope.replay.proto_logger import ProtoLogger

NUM_ROBOTS = 6
SIM_TICK_RATE_MS = 16

###########################################################################
#                         Thunderscope Main                               #
###########################################################################

if __name__ == "__main__":

    # Setup parser
    parser = argparse.ArgumentParser(
        description="Thunderscope: Run with no arguments to run AI vs AI"
    )

    parser.add_argument(
        "--layout",
        action="store",
        help="Which layout to run, if not specified the last layout will run",
    )

    # Runtime directories
    parser.add_argument(
        "--simulator_runtime_dir",
        type=str,
        help="simulator runtime directory",
        default="/tmp/tbots/sim",
    )
    parser.add_argument(
        "--blue_full_system_runtime_dir",
        type=str,
        help="blue full_system runtime directory",
        default="/tmp/tbots/blue",
    )
    parser.add_argument(
        "--yellow_full_system_runtime_dir",
        type=str,
        help="yellow full_system runtime directory",
        default="/tmp/tbots/yellow",
    )

    # Debugging
    parser.add_argument(
        "--debug_blue_full_system",
        action="store_true",
        default=False,
        help="Debug blue full_system",
    )
    parser.add_argument(
        "--debug_yellow_full_system",
        action="store_true",
        default=False,
        help="Debug yellow full_system",
    )
    parser.add_argument(
        "--debug_simulator",
        action="store_true",
        default=False,
        help="Debug the simulator",
    )
    parser.add_argument(
        "--visualize_cpp_test",
        action="store_true",
        default=False,
        help="Visualize C++ Tests",
    )
    parser.add_argument(
        "--blue_log",
        action="store",
        help="Replay folder for the blue full_system",
        default=None,
        type=os.path.abspath,
    )
    parser.add_argument(
        "--yellow_log",
        action="store",
        help="Replay folder for the yellow full_system",
        default=None,
    )
    # Run blue or yellow full system over WiFi
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--run_blue",
        action="store_true",
        help="Run full system as the blue team, over WiFi; estop required",
    )
    group.add_argument(
        "--run_yellow",
        action="store_true",
        help="Run full system as the yellow team, over WiFi; estop required",
    )
    parser.add_argument(
        "--interface",
        action="store",
        type=str,
        default=None,
        help="Which interface to communicate over",
    )
    parser.add_argument(
        "--channel",
        action="store",
        type=int,
        default=0,
        help="Which channel to communicate over",
    )
    parser.add_argument(
        "--visualization_buffer_size",
        action="store",
        type=int,
        default=5,
        help="How many packets to buffer while rendering",
    )
    parser.add_argument(
        "--enable_realism",
        action="store_true",
        default=False,
        help="set realism flag to use realistic config",
    )
    parser.add_argument(
        "--estop_path",
        action="store",
        type=str,
        default="/dev/ttyACM0",
        help="Path to the Estop",
    )
    parser.add_argument(
        "--estop_baudrate",
        action="store",
        type=int,
        default=115200,
        help="Estop Baudrate",
    )

    # Sanity check that an interface was provided
    args = parser.parse_args()

    if args.run_blue or args.run_yellow:
        if args.interface is None:
            parser.error("Must specify interface")

    ###########################################################################
    #                      Visualize CPP Tests                                #
    ###########################################################################
    # TODO (#2581) remove this
    if args.visualize_cpp_test:

        runtime_dir = "/tmp/tbots/yellow_test"

        try:
            os.makedirs(runtime_dir)
        except OSError:
            pass

        tscope = Thunderscope(
            layout_path=args.layout,
            visualization_buffer_size=args.visualization_buffer_size,
        )
        proto_unix_io = tscope.blue_full_system_proto_unix_io

        # Setup LOG(VISUALIZE) handling from full system. We set from_log_visualize
        # to true to decode from base64.
        for arg in [
            {"proto_class": Obstacles},
            {"proto_class": PathVisualization},
            {"proto_class": PassVisualization},
            {"proto_class": NamedValue},
            {"proto_class": PrimitiveSet},
            {"proto_class": World},
            {"proto_class": PlayInfo},
        ] + [
            # TODO (#2655): Add/Remove HRVO layers dynamically based on the HRVOVisualization proto messages
            {"proto_class": HRVOVisualization, "unix_path": YELLOW_HRVO_PATH}
            for _ in range(MAX_ROBOT_IDS_PER_SIDE)
        ]:
            proto_unix_io.attach_unix_receiver(
                runtime_dir, from_log_visualize=True, **arg
            )

        proto_unix_io.attach_unix_receiver(runtime_dir + "/log", proto_class=RobotLog)

        tscope.show()

    ###########################################################################
    #              AI + Robot Communication + Robot Diagnostics               #
    ###########################################################################
    #
    # When we are running with real robots. We want to run 1 instance of AI
    # and 1 instance of RobotCommunication which will send/recv packets over
    # the provided multicast channel.
    if args.run_blue:

        tscope = Thunderscope(
            layout_path=args.layout,
            load_blue=True,
            load_yellow=False,
            load_diagnostics=True,
            load_gamecontroller=False,
            visualization_buffer_size=args.visualization_buffer_size,
        )

        proto_unix_io = tscope.blue_full_system_proto_unix_io
        runtime_dir = args.blue_full_system_runtime_dir
        friendly_colour_yellow = False
        debug = args.debug_blue_full_system
    elif args.run_yellow:

        tscope = Thunderscope(
            layout_path=args.layout,
            load_blue=False,
            load_yellow=True,
            load_diagnostics=True,
            load_gamecontroller=False,
            visualization_buffer_size=args.visualization_buffer_size,
        )

        proto_unix_io = tscope.yellow_full_system_proto_unix_io
        runtime_dir = args.yellow_full_system_runtime_dir
        friendly_colour_yellow = True
        debug = args.debug_yellow_full_system

    if args.run_blue or args.run_yellow:
        with ProtoLogger(
            args.blue_full_system_runtime_dir,
        ) as blue_logger, ProtoLogger(
            args.yellow_full_system_runtime_dir,
        ) as yellow_logger, RobotCommunication(
            proto_unix_io, getRobotMulticastChannel(0), args.interface
        ), FullSystem(
            runtime_dir, debug, friendly_colour_yellow
        ) as full_system:

            proto_unix_io.register_to_observe_everything(blue_logger.buffer)
            proto_unix_io.register_to_observe_everything(yellow_logger.buffer)
            full_system.setup_proto_unix_io(proto_unix_io)
            tscope.show()

    ###########################################################################
    #                              Replay                                     #
    ###########################################################################
    #
    # Don't start any binaries and just replay a log.
    elif args.blue_log or args.yellow_log:
        tscope = Thunderscope(
            layout_path=args.layout,
            visualization_buffer_size=args.visualization_buffer_size,
            blue_replay_log=args.blue_log,
            yellow_replay_log=args.yellow_log,
        )
        tscope.show()

    ###########################################################################
    #           Blue AI vs Yellow AI + Simulator + Gamecontroller             #
    ###########################################################################
    #
    # Run two AIs against each other with the er-force simulator. We also run
    # the gamecontroller which can be accessed from http://localhost:8081
    #
    # The async sim ticket ticks the simulator at a fixed rate.
    else:

        tscope = Thunderscope(
            layout_path=args.layout,
            visualization_buffer_size=args.visualization_buffer_size,
        )

        def __async_sim_ticker(tick_rate_ms):
            """Setup the world and tick simulation forever

            :param tick_rate_ms: The tick rate of the simulation

            """
            world_state = tbots_protobuf.create_world_state(
                blue_robot_locations=[
                    cpp_bindings.Point(-3, y) for y in numpy.linspace(-2, 2, NUM_ROBOTS)
                ],
                yellow_robot_locations=[
                    cpp_bindings.Point(3, y) for y in numpy.linspace(-2, 2, NUM_ROBOTS)
                ],
                ball_location=cpp_bindings.Point(0, 0),
                ball_velocity=cpp_bindings.Vector(0, 0),
            )
            tscope.simulator_proto_unix_io.send_proto(WorldState, world_state)

            # Tick Simulation
            while True:
                tick = SimulatorTick(milliseconds=tick_rate_ms)
                tscope.simulator_proto_unix_io.send_proto(SimulatorTick, tick)
                time.sleep(tick_rate_ms / 1000)

        # Launch all binaries
        with Simulator(
            args.simulator_runtime_dir, args.debug_simulator, args.enable_realism
        ) as simulator, FullSystem(
            args.blue_full_system_runtime_dir, args.debug_blue_full_system, False
        ) as blue_fs, FullSystem(
            args.yellow_full_system_runtime_dir, args.debug_yellow_full_system, True
        ) as yellow_fs, ProtoLogger(
            args.blue_full_system_runtime_dir,
        ) as blue_logger, ProtoLogger(
            args.yellow_full_system_runtime_dir,
        ) as yellow_logger, Gamecontroller() as gamecontroller:

            tscope.blue_full_system_proto_unix_io.register_to_observe_everything(
                blue_logger.buffer
            )
            tscope.yellow_full_system_proto_unix_io.register_to_observe_everything(
                yellow_logger.buffer
            )

            blue_fs.setup_proto_unix_io(tscope.blue_full_system_proto_unix_io)
            yellow_fs.setup_proto_unix_io(tscope.yellow_full_system_proto_unix_io)
            simulator.setup_proto_unix_io(
                tscope.simulator_proto_unix_io,
                tscope.blue_full_system_proto_unix_io,
                tscope.yellow_full_system_proto_unix_io,
            )
            gamecontroller.setup_proto_unix_io(
                tscope.blue_full_system_proto_unix_io,
                tscope.yellow_full_system_proto_unix_io,
            )

            # Start the simulator
            thread = threading.Thread(
                target=__async_sim_ticker, args=(SIM_TICK_RATE_MS,), daemon=True,
            )

            thread.start()
            tscope.show()
            thread.join()
