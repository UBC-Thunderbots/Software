from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.binary_context_managers import *
from software.thunderscope.replay.proto_logger import ProtoLogger
from proto.message_translation import tbots_protobuf
import software.python_bindings as cpp_bindings
from proto.ssl_gc_common_pb2 import Team

import numpy

SIM_TICK_RATE_MS = 16

def start_ai_vs_ai(simulator_runtime_dir, blue_fs_dir, yellow_fs_dir):
    def __async_sim_ticker(simulator_proto_unix_io):
        """Setup the world and tick simulation forever

        :param tick_rate_ms: The tick rate of the simulation

        """
        world_state = tbots_protobuf.create_world_state(
            blue_robot_locations=[
                cpp_bindings.Point(-3, y) for y in numpy.linspace(-2, 2, DIV_B_NUM_ROBOTS)
            ],
            yellow_robot_locations=[
                cpp_bindings.Point(3, y) for y in numpy.linspace(-2, 2, DIV_B_NUM_ROBOTS)
            ],
            ball_location=cpp_bindings.Point(0, 0),
            ball_velocity=cpp_bindings.Vector(0, 0),
        )
        simulator_proto_unix_io.send_proto(WorldState, world_state)

        # Tick Simulation
        while True:
            tick = SimulatorTick(milliseconds=SIM_TICK_RATE_MS)
            simulator_proto_unix_io.send_proto(SimulatorTick, tick)

    blue_fs_proto_unix_io   = ProtoUnixIO()
    yellow_fs_proto_unix_io = ProtoUnixIO()
    simulator_proto_unix_io = ProtoUnixIO()

    with Simulator(
            simulator_runtime_dir
    ) as simulator, FullSystem(
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
    ) as autoref:
        blue_fs_proto_unix_io.register_to_observe_everything(
                blue_logger.buffer
        )
        yellow_fs_proto_unix_io.register_to_observe_everything(
                yellow_logger.buffer
        )

        blue_fs.setup_proto_unix_io(blue_fs_proto_unix_io)
        yellow_fs.setup_proto_unix_io(yellow_fs_proto_unix_io)
        simulator.setup_proto_unix_io(
                simulator_proto_unix_io,
                blue_fs_proto_unix_io,
                yellow_fs_proto_unix_io
        )
        gamecontroller.setup_proto_unix_io(
                blue_fs_proto_unix_io,
                yellow_fs_proto_unix_io
        )

        autoref_proto_unix_io = ProtoUnixIO()
        simulator.setup_autoref_proto_unix_io(autoref_proto_unix_io)
        autoref.setup_ssl_wrapper_packets(autoref_proto_unix_io, blue_fs_proto_unix_io, yellow_fs_proto_unix_io)

        thread = threading.Thread(
            target=__async_sim_ticker, args=(simulator_proto_unix_io,), daemon=True,
        )
        thread.start()
        thread.join()

        
if __name__ == "__main__":
    start_ai_vs_ai("/tmp/tbots/sim", "/tmp/tbots/blue", "/tmp/tbots/yellow")

