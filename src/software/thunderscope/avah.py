import sys
import time
import threading

import software.python_bindings as tbots_cpp

import proto.import_all_protos as protos
from extlibs.er_force_sim.src.protobuf import world_pb2 as er_force
from proto.message_translation.tbots_protobuf import create_world_state
from software.networking.unix.threaded_unix_listener import ThreadedUnixListener
from software.networking.unix.threaded_unix_sender import ThreadedUnixSender
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer


def low_level():
    listener = ThreadedUnixListener("/tmp/avah/woo", protos.Angle)
    threading.Thread(target=listener.start, daemon=True).start()

    sender = ThreadedUnixSender("/tmp/avah/woo", protos.Angle)
    sender.send(protos.Angle(radians=0.1))

    while True:
        if not listener.proto_buffer.empty():
            print("not empty!")
            a = listener.proto_buffer.get()
            print(a)


if __name__ == "__main__":
    received_buf = ThreadSafeBuffer(
        buffer_size=20, protobuf_type=protos.WorldStateReceivedTrigger
    )
    sim_state_buf = ThreadSafeBuffer(
        buffer_size=20, protobuf_type=er_force.SimulatorState
    )

    puio = ProtoUnixIO()
    puio.attach_unix_receiver(
        "/tmp/avah/", "world_state_received_trigger", protos.WorldStateReceivedTrigger
    )
    puio.register_observer(protos.WorldStateReceivedTrigger, received_buf)

    puio.attach_unix_receiver("/tmp/avah/", "simulator_state", er_force.SimulatorState)
    puio.register_observer(er_force.SimulatorState, sim_state_buf)

    puio.attach_unix_sender("/tmp/avah/", "world_state", protos.WorldState)
    puio.attach_unix_sender("/tmp/avah/", "simulation_tick", protos.SimulatorTick)

    init_world_state = create_world_state(
        yellow_robot_locations=[],
        blue_robot_locations=[],
        ball_location=tbots_cpp.Point(1, 0),
        ball_velocity=tbots_cpp.Vector(-1, 0),
    )

    puio.send_proto(protos.WorldState, init_world_state)

    # if not received_buf.get(return_cached=False):
    #     sys.exit(1)

    time.sleep(0.1)
    puio.send_proto(protos.SimulatorTick, protos.SimulatorTick(milliseconds=10))
    time.sleep(0.1)
    puio.send_proto(protos.SimulatorTick, protos.SimulatorTick(milliseconds=10))
    time.sleep(0.1)
    puio.send_proto(protos.SimulatorTick, protos.SimulatorTick(milliseconds=10))

    time.sleep(1)

    while True:
        state = sim_state_buf.get(return_cached=False)
        if state:
            print("received state:", state)
        else:
            print("no state, ending")
            break
