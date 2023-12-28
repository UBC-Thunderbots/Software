from proto.import_all_protos import *
from proto.message_translation import tbots_protobuf
import software.python_bindings as tbots_cpp
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.thunderscope import Thunderscope

import numpy
import time


def async_sim_ticker(
    tick_rate_ms: int,
    blue_proto_unix_io: ProtoUnixIO,
    yellow_proto_unix_io: ProtoUnixIO,
    sim_proto_unix_io: ProtoUnixIO,
) -> None:
    """
    Tick simulation as fast as possible, waiting for the Blue and Yellow AIs to process the vision packet before ticking next.

    :param tick_rate_ms:            the interval between consequent ticks (ms) 
    :param blue_proto_unix_io:      ProtoUnixIO for the Blue FullSystem 
    :param yellow_proto_unix_io:    ProtoUnixIO for the Yellow FullSystem
    :param sim_proto_unix_io:       ProtoUnixIO for the Simulation
    """
    blue_primitive_set_buffer = ThreadSafeBuffer(
        buffer_size=1, protobuf_type=PrimitiveSet
    )
    yellow_primitive_set_buffer = ThreadSafeBuffer(
        buffer_size=1, protobuf_type=PrimitiveSet
    )

    blue_proto_unix_io.register_observer(PrimitiveSet, blue_primitive_set_buffer)
    yellow_proto_unix_io.register_observer(PrimitiveSet, yellow_primitive_set_buffer)

    while True:
        # Tick simulation
        tick = SimulatorTick(milliseconds=tick_rate_ms)
        sim_proto_unix_io.send_proto(SimulatorTick, tick)

        blue_primitive_set_buffer.get(block=True)
        yellow_primitive_set_buffer.get(block=True)


def realtime_sim_ticker(
    tick_rate_ms: int, sim_proto_unix_io: ProtoUnixIO, tscope: Thunderscope
) -> None:
    """
    Tick simulation in real-time. Requires Thunderscope to be open.

    :param tick_rate_ms:        the interval between consequent ticks (ms) and delay between sending Vision messages
    :param sim_proto_unix_io:   ProtoUnixIO for the Simulation
    :param tscope:              Thunderscope instance that is tied to the simulation ticking
    """
    simulation_state_buffer = ThreadSafeBuffer(1, SimulationState)
    sim_proto_unix_io.register_observer(SimulationState, simulation_state_buffer)

    # Tick simulation if Thundersocpe is open
    while tscope.is_open():
        simulation_state_message = simulation_state_buffer.get()

        if simulation_state_message.is_playing:
            tick = SimulatorTick(milliseconds=tick_rate_ms)
            sim_proto_unix_io.send_proto(SimulatorTick, tick)

        time.sleep(tick_rate_ms / 1000)


def sync_simulation(sim_proto_unix_io: ProtoUnixIO, num_robots: int) -> None:
    """
    Ensure that simulator has synchronized with the default world state.

    :param sim_proto_unix_io:   ProtoUnixIO for the Simulation
    :param num_robots:          Number of robots to initialize the simulator with
    """
    world_state_received_buffer = ThreadSafeBuffer(1, WorldStateReceivedTrigger)
    sim_proto_unix_io.register_observer(
        WorldStateReceivedTrigger, world_state_received_buffer
    )

    while True:
        world_state_received = world_state_received_buffer.get(
            block=False, return_cached=False
        )
        if not world_state_received:
            world_state = tbots_protobuf.create_default_world_state(num_robots)
            sim_proto_unix_io.send_proto(WorldState, world_state)
        else:
            break
