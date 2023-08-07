from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.binary_context_managers import *
from software.thunderscope.replay.proto_logger import ProtoLogger
from proto.message_translation import tbots_protobuf
import software.python_bindings as cpp_bindings

import numpy


def start_ai_vs_ai(simulator_runtime_dir, blue_fs_dir, yellow_fs_dir):
    """
    Start AI vs AI silently, ticking as fast as possible without visualization.

    Note: This module is different from Thunderscope as it does not run any Qt elements at all. It runs two
    FullSystems, a Gamecontroller and TigersAutoref. Qt does not work with display-less devices, so it won't work
    properly in a CI setting, which is why this AI vs AI setup is required.

    :param simulator_runtime_dir    the runtime directory to set up ProtoUnixIO for the Simulator
    :param blue_fs_dir              the runtime directory to set up ProtoUnixIO for the blue FullSystem
    :param yellow_fs_dir            the runtime directory to set up ProtoUnixIO for the yellow FullSystem
    """

    def __async_sim_ticker(simulator_proto_unix_io):
        """Setup the world and tick simulation forever, as fast as possible

        :param tick_rate_ms: The tick rate of the simulation

        """
        world_state_received_buffer = ThreadSafeBuffer(1, WorldStateReceivedTrigger)
        simulator_proto_unix_io.register_observer(
            WorldStateReceivedTrigger, world_state_received_buffer
        )

        while True:
            world_state_received = world_state_received_buffer.get(
                block=False, return_cached=False
            )
            if not world_state_received:
                world_state = tbots_protobuf.create_default_world_state(
                    DIV_B_NUM_ROBOTS
                )
                simulator_proto_unix_io.send_proto(WorldState, world_state)
            else:
                break

            time.sleep(0.01)

        # Tick Simulation
        tick = SimulatorTick(
            milliseconds=DEFAULT_SIMULATOR_TICK_RATE_MILLISECONDS_PER_TICK
        )
        simulator_proto_unix_io.send_proto(SimulatorTick, tick)

    blue_fs_proto_unix_io = ProtoUnixIO()
    yellow_fs_proto_unix_io = ProtoUnixIO()
    simulator_proto_unix_io = ProtoUnixIO()

    with Simulator(simulator_runtime_dir) as simulator, FullSystem(
        blue_fs_dir, friendly_colour_yellow=False
    ) as blue_fs, FullSystem(
        yellow_fs_dir, friendly_colour_yellow=True
    ) as yellow_fs, ProtoLogger(
        f"{blue_fs_dir}/logs/",
    ) as blue_logger, ProtoLogger(
        f"{yellow_fs_dir}/logs/",
    ) as yellow_logger, Gamecontroller(
        ci_mode=True
    ) as gamecontroller, TigersAutoref(
        autoref_runtime_dir="/tmp/tbots/autoref",
        ci_mode=True,
        gc=gamecontroller,
        tick_rate_ms=DEFAULT_SIMULATOR_TICK_RATE_MILLISECONDS_PER_TICK,
    ) as autoref:
        autoref_proto_unix_io = ProtoUnixIO()

        blue_fs_proto_unix_io.register_to_observe_everything(blue_logger.buffer)
        yellow_fs_proto_unix_io.register_to_observe_everything(yellow_logger.buffer)

        blue_fs.setup_proto_unix_io(blue_fs_proto_unix_io)
        yellow_fs.setup_proto_unix_io(yellow_fs_proto_unix_io)
        simulator.setup_proto_unix_io(
            simulator_proto_unix_io,
            blue_fs_proto_unix_io,
            yellow_fs_proto_unix_io,
            autoref_proto_unix_io,
        )
        gamecontroller.setup_proto_unix_io(
            blue_fs_proto_unix_io, yellow_fs_proto_unix_io, autoref_proto_unix_io
        )

        autoref.setup_ssl_wrapper_packets(autoref_proto_unix_io)

        thread = threading.Thread(
            target=__async_sim_ticker, args=(simulator_proto_unix_io,), daemon=True,
        )
        thread.start()
        thread.join()


if __name__ == "__main__":
    start_ai_vs_ai("/tmp/tbots/sim", "/tmp/tbots/blue", "/tmp/tbots/yellow")
