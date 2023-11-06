import os
import time
import threading
import argparse
import contextlib

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.binary_context_managers import *
from proto.message_translation import tbots_protobuf
import software.python_bindings as tbots_cpp
from software.py_constants import *
from software.thunderscope.robot_communication import RobotCommunication
from software.thunderscope.replay.proto_logger import ProtoLogger
from software.thunderscope.proto_unix_io import ProtoUnixIO
import software.thunderscope.thunderscope_config as config
from software.thunderscope.constants import ProtoUnixIOTypes

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

    parser.add_argument(
        "--verbose",
        action="store_true",
        default=False,
        help="Include logs from the Gamecontroller and Autoref",
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
        "--run_diagnostics",
        action="store_true",
        help="Run robots diagnostics for Manual or Xbox control; estop required",
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
    parser.add_argument(
        "--enable_autoref", action="store_true", default=False, help="Enable autoref"
    )
    parser.add_argument(
        "--show_autoref_gui",
        action="store_true",
        default=False,
        help="Show TigersAutoref GUI",
    )
    parser.add_argument(
        "--cost_visualization",
        action="store_true",
        help="show pass cost visualization layer",
    )
    parser.add_argument(
        "--disable_estop",
        action="store_true",
        default=False,
        help="Disables checking for estop plugged in (ONLY USE FOR LOCAL TESTING)",
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
            config=config.configure_two_ai_gamecontroller_view(
                args.visualization_buffer_size, args.cost_visualization
            ),
            layout_path=args.layout,
        )
        proto_unix_io = tscope.proto_unix_io_map[ProtoUnixIOTypes.BLUE]

        # Setup LOG(VISUALIZE) handling from full system. We set from_log_visualize
        # to true to decode from base64.
        for arg in [
            {"proto_class": Obstacles},
            {"proto_class": PathVisualization},
            {"proto_class": PassVisualization},
            {"proto_class": CostVisualization},
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
    # When we are running with real robots. Possible Running Options:
    #
    #       Run Blue
    #       Run Yellow
    #       Run Blue + Diagnostics
    #       Run Yellow + Diagnostics
    #       Run Diagnostics
    #
    # We want to run either 1 instance of AI or 1 instance of RobotCommunication or both which will
    # send/recv packets over the provided multicast channel.

    elif args.run_blue or args.run_yellow or args.run_diagnostics:
        tscope_config = config.configure_ai_or_diagnostics(
            args.run_blue,
            args.run_yellow,
            args.run_diagnostics,
            args.visualization_buffer_size,
            args.cost_visualization,
        )
        tscope = Thunderscope(config=tscope_config, layout_path=args.layout,)

        current_proto_unix_io = None

        if args.run_blue:
            runtime_dir = args.blue_full_system_runtime_dir
            friendly_colour_yellow = False
            debug = args.debug_blue_full_system
        elif args.run_yellow:
            runtime_dir = args.yellow_full_system_runtime_dir
            friendly_colour_yellow = True
            debug = args.debug_yellow_full_system

        # this will be the current fullsystem proto (blue or yellow)
        # if fullsystem is loaded
        # else, it will be the diagnostics proto
        current_proto_unix_io = tscope.proto_unix_io_map[ProtoUnixIOTypes.CURRENT]

        # different estops use different ports this detects which one to use based on what is plugged in
        estop_path = (
            "/dev/ttyACM0" if os.path.isfile("/dev/ttyACM0") else "/dev/ttyUSB0"
        )

        with RobotCommunication(
            current_proto_unix_io,
            getRobotMulticastChannel(args.channel),
            args.interface,
            args.disable_estop,
            estop_path,
        ) as robot_communication:
            if args.run_diagnostics:
                for tab in tscope_config.tabs:
                    if hasattr(tab, "widgets"):
                        robot_view_widget = tab.find_widget("Robot View")

                        if robot_view_widget:
                            robot_view_widget.control_mode_signal.connect(
                                lambda mode, robot_id: robot_communication.toggle_robot_connection(
                                    mode, robot_id
                                )
                            )

            if args.run_blue or args.run_yellow:
                robot_communication.setup_for_fullsystem()
                full_system_runtime_dir = (
                    args.blue_full_system_runtime_dir
                    if args.run_blue
                    else args.yellow_full_system_runtime_dir
                )
                with ProtoLogger(full_system_runtime_dir,) as logger, FullSystem(
                    runtime_dir, debug, friendly_colour_yellow
                ) as full_system:

                    current_proto_unix_io.register_to_observe_everything(logger.buffer)
                    full_system.setup_proto_unix_io(current_proto_unix_io)

                    tscope.show()
            else:
                tscope.show()

    ###########################################################################
    #                              Replay                                     #
    ###########################################################################
    #
    # Don't start any binaries and just replay a log.
    elif args.blue_log or args.yellow_log:
        tscope = Thunderscope(
            config=config.configure_replay_view(
                args.blue_log,
                args.yellow_log,
                args.visualization_buffer_size,
                args.cost_visualization,
            ),
            layout_path=args.layout,
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
            config=config.configure_two_ai_gamecontroller_view(
                args.visualization_buffer_size, args.cost_visualization
            ),
            layout_path=args.layout,
        )

        def __async_sim_ticker(tick_rate_ms: int) -> None:
            """Setup the world and tick simulation forever

            :param tick_rate_ms: The tick rate of the simulation

            """
            world_state_received_buffer = ThreadSafeBuffer(1, WorldStateReceivedTrigger)
            tscope.proto_unix_io_map[ProtoUnixIOTypes.SIM].register_observer(
                WorldStateReceivedTrigger, world_state_received_buffer
            )

            while True:
                world_state_received = world_state_received_buffer.get(
                    block=False, return_cached=False
                )
                if not world_state_received:
                    world_state = tbots_protobuf.create_world_state(
                        blue_robot_locations=[
                            tbots_cpp.Point(-3, y)
                            for y in numpy.linspace(-2, 2, NUM_ROBOTS)
                        ],
                        yellow_robot_locations=[
                            tbots_cpp.Point(3, y)
                            for y in numpy.linspace(-2, 2, NUM_ROBOTS)
                        ],
                        ball_location=tbots_cpp.Point(0, 0),
                        ball_velocity=tbots_cpp.Vector(0, 0),
                    )
                    tscope.proto_unix_io_map[ProtoUnixIOTypes.SIM].send_proto(
                        WorldState, world_state
                    )
                else:
                    break

                time.sleep(0.01)

            simulation_state_buffer = ThreadSafeBuffer(1, SimulationState)
            tscope.proto_unix_io_map[ProtoUnixIOTypes.SIM].register_observer(
                SimulationState, simulation_state_buffer
            )

            # Tick Simulation
            while tscope.is_open():

                simulation_state_message = simulation_state_buffer.get()

                if simulation_state_message.is_playing:
                    tick = SimulatorTick(milliseconds=tick_rate_ms)
                    tscope.proto_unix_io_map[ProtoUnixIOTypes.SIM].send_proto(
                        SimulatorTick, tick
                    )

                time.sleep(tick_rate_ms / 1000)

        # Launch all binaries
        with Simulator(
            args.simulator_runtime_dir, args.debug_simulator, args.enable_realism
        ) as simulator, FullSystem(
            args.blue_full_system_runtime_dir, args.debug_blue_full_system, False, False
        ) as blue_fs, FullSystem(
            args.yellow_full_system_runtime_dir,
            args.debug_yellow_full_system,
            True,
            False,
        ) as yellow_fs, ProtoLogger(
            args.blue_full_system_runtime_dir,
        ) as blue_logger, ProtoLogger(
            args.yellow_full_system_runtime_dir,
        ) as yellow_logger, Gamecontroller(
            supress_logs=(not args.verbose), ci_mode=args.enable_autoref
        ) as gamecontroller, (
            # Here we only initialize autoref if the --enable_autoref flag is requested.
            # To avoid nested Python withs, the autoref is initialized as None when this flag doesn't exist.
            # All calls to autoref should be guarded with args.enable_autoref
            TigersAutoref(
                autoref_runtime_dir="/tmp/tbots/autoref",
                ci_mode=True,
                gc=gamecontroller,
                supress_logs=(not args.verbose),
                tick_rate_ms=DEFAULT_SIMULATOR_TICK_RATE_MILLISECONDS_PER_TICK,
                show_gui=args.show_autoref_gui,
            )
            if args.enable_autoref
            else contextlib.nullcontext()
        ) as autoref:
            autoref_proto_unix_io = ProtoUnixIO()

            tscope.proto_unix_io_map[
                ProtoUnixIOTypes.BLUE
            ].register_to_observe_everything(blue_logger.buffer)
            tscope.proto_unix_io_map[
                ProtoUnixIOTypes.YELLOW
            ].register_to_observe_everything(yellow_logger.buffer)

            blue_fs.setup_proto_unix_io(tscope.proto_unix_io_map[ProtoUnixIOTypes.BLUE])
            yellow_fs.setup_proto_unix_io(
                tscope.proto_unix_io_map[ProtoUnixIOTypes.YELLOW]
            )
            simulator.setup_proto_unix_io(
                tscope.proto_unix_io_map[ProtoUnixIOTypes.SIM],
                tscope.proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                tscope.proto_unix_io_map[ProtoUnixIOTypes.YELLOW],
                autoref_proto_unix_io,
            )
            gamecontroller.setup_proto_unix_io(
                tscope.proto_unix_io_map[ProtoUnixIOTypes.BLUE],
                tscope.proto_unix_io_map[ProtoUnixIOTypes.YELLOW],
                autoref_proto_unix_io,
            )
            if args.enable_autoref:
                autoref.setup_ssl_wrapper_packets(autoref_proto_unix_io,)

            # Start the simulator
            thread = threading.Thread(
                target=__async_sim_ticker,
                args=(DEFAULT_SIMULATOR_TICK_RATE_MILLISECONDS_PER_TICK,),
                daemon=True,
            )

            thread.start()
            tscope.show()
            thread.join()
