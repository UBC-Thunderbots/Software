import platform
from typing import Callable, NoReturn, TYPE_CHECKING

if TYPE_CHECKING:
    from software.thunderscope.thunderscope import Thunderscope

from proto.import_all_protos import *
from proto.message_translation import tbots_protobuf
from software.py_constants import SECONDS_PER_MILLISECOND
from software.thunderscope.constants import ProtoUnixIOTypes
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from software.thunderscope.time_provider import TimeProvider

import queue
import time

from pyqtgraph.Qt import QtGui
import software.python_bindings as tbots_cpp


def exit_poller(
    time_provider: TimeProvider,
    exit_duration_s: float,
    on_exit: Callable[[], None],
    poll_duration_s: float = 0.5,
) -> NoReturn:
    """Calls the on_exit callback once the elapsed exit_duration has passed from the start of this function call,
    polling every poll_duration using system time

    :param time_provider:   used to compare all timestamps
    :param exit_duration_s: how long to poll from the start of this program before exiting
    :param on_exit:         callback once the exit_duration time has elapsed
    :param poll_duration_s: interval between polling the time_provider for the current timestamp
    """
    time_now_s = time_provider.time_provider()
    while time_provider.time_provider() <= (time_now_s + exit_duration_s):
        time.sleep(poll_duration_s)

    on_exit()


def async_sim_ticker(
    tick_rate_ms: int,
    blue_proto_unix_io: ProtoUnixIO,
    yellow_proto_unix_io: ProtoUnixIO,
    sim_proto_unix_io: ProtoUnixIO,
    tscope: "Thunderscope",
    buffer_timeout_s: int = 1,
) -> None:
    """Tick simulation as fast as possible, waiting for the Blue and Yellow AIs to process the vision packet before ticking next.

    :param tick_rate_ms:            the interval between consequent ticks (ms)
    :param blue_proto_unix_io:      ProtoUnixIO for the Blue FullSystem
    :param yellow_proto_unix_io:    ProtoUnixIO for the Yellow FullSystem
    :param sim_proto_unix_io:       ProtoUnixIO for the Simulation
    :param tscope:                  Thunderscope instance that is tied to the simulation ticking
    :param buffer_timeout_s:        How long to wait for a response from the AI before assuming that the AI missed the
                                    SSL Vision packet
    """
    blue_primitive_set_buffer = ThreadSafeBuffer(
        buffer_size=1, protobuf_type=PrimitiveSet
    )
    yellow_primitive_set_buffer = ThreadSafeBuffer(
        buffer_size=1, protobuf_type=PrimitiveSet
    )

    blue_proto_unix_io.register_observer(PrimitiveSet, blue_primitive_set_buffer)
    yellow_proto_unix_io.register_observer(PrimitiveSet, yellow_primitive_set_buffer)

    while tscope.is_open():
        # flush primitive set buffers before sending the next tick
        while (
            blue_primitive_set_buffer.get(block=False, return_cached=False) is not None
            or yellow_primitive_set_buffer.get(block=False, return_cached=False)
            is not None
        ):
            pass

        # Tick simulation
        tick = SimulatorTick(milliseconds=tick_rate_ms)
        sim_proto_unix_io.send_proto(SimulatorTick, tick)

        while True:
            try:
                blue_primitive_set_buffer.get(block=True, timeout=buffer_timeout_s)
                break
            except queue.Empty:
                sim_proto_unix_io.send_proto(SimulatorTick, tick)

        while True:
            try:
                yellow_primitive_set_buffer.get(block=True, timeout=buffer_timeout_s)
                break
            except queue.Empty:
                sim_proto_unix_io.send_proto(SimulatorTick, tick)


def realtime_sim_ticker(
    tick_rate_ms: int, sim_proto_unix_io: ProtoUnixIO, tscope: "Thunderscope"
) -> None:
    """Tick simulation in real-time. Requires Thunderscope to be open.

    :param tick_rate_ms:        the interval between consequent ticks (ms) and delay between sending Vision messages
    :param sim_proto_unix_io:   ProtoUnixIO for the Simulation
    :param tscope:              Thunderscope instance that is tied to the simulation ticking
    """
    simulation_state_buffer = ThreadSafeBuffer(5, SimulationState)
    sim_proto_unix_io.register_observer(SimulationState, simulation_state_buffer)
    per_tick_delay_s = tick_rate_ms * SECONDS_PER_MILLISECOND

    # Tick simulation if Thundersocpe is open
    while tscope.is_open():
        simulation_state_message = simulation_state_buffer.get()

        if simulation_state_message.is_playing:
            tick = SimulatorTick(milliseconds=tick_rate_ms)
            sim_proto_unix_io.send_proto(SimulatorTick, tick)

        time.sleep(per_tick_delay_s / simulation_state_message.simulation_speed)


def sync_simulation(
    tscope: "Thunderscope", num_robots: int, timeout_s: float = 0.1
) -> None:
    """Ensure that simulator has synchronized with the default world state.

    :param tscope:              Thunderscope instance that is tied to this instance of the simulation
    :param num_robots:          Number of robots to initialize the simulator with
    :param timeout_s:           How long to wait before we retry our attempt to synchronize with the simulator
    """
    sim_proto_unix_io = tscope.proto_unix_io_map[ProtoUnixIOTypes.SIM]
    world_state_received_buffer = ThreadSafeBuffer(1, WorldStateReceivedTrigger)
    sim_proto_unix_io.register_observer(
        WorldStateReceivedTrigger, world_state_received_buffer
    )
    world_state = tbots_protobuf.create_default_world_state(num_robots)

    while True:
        sim_proto_unix_io.send_proto(WorldState, world_state)

        try:
            world_state_received = world_state_received_buffer.get(
                block=True, timeout=timeout_s
            )
        except queue.Empty:
            # Did not receive a response within timeout period
            continue
        else:
            # Received a response from the simulator
            break

    # Wait for Thunderscope to launch
    while not tscope.is_open():
        time.sleep(timeout_s)


def color_from_gradient(
    x: float,
    t_range: list[float],
    r_range: list[int],
    g_range: list[int],
    b_range: list[int],
    a_range: list[int],
):
    """Returns a color interpolated from a gradient defined by the point along a sigmoid curve.
    :param x: value to interpolate
    :param t_range: the thresholds for each section in the gradient
    :param r_range: the r values at each threshold
    :param g_range: the g values at each threshold
    :param b_range: the b values at each threshold
    :param a_range: the a values at each threshold
    :return: a color according to the gradient
    """
    if not (
        len(t_range) == len(r_range) == len(g_range) == len(b_range) == len(a_range)
    ):
        return QtGui.QColor(255, 255, 255, 255)
    else:
        if x < t_range[0]:
            return QtGui.QColor(r_range[0], g_range[0], b_range[0], a_range[0])
        elif x > t_range[-1]:
            return QtGui.QColor(r_range[-1], g_range[-1], b_range[-1], a_range[-1])
        else:
            for i in range(len(t_range) - 1):
                if t_range[i] <= x < t_range[i + 1]:
                    sig_val = tbots_cpp.sigmoid(
                        x,
                        (t_range[i] + t_range[i + 1]) / 2,
                        t_range[i + 1] - t_range[i],
                    )
                    return QtGui.QColor(
                        int(r_range[i] + (r_range[i + 1] - r_range[i]) * sig_val),
                        int(g_range[i] + (g_range[i + 1] - g_range[i]) * sig_val),
                        int(b_range[i] + (b_range[i + 1] - b_range[i]) * sig_val),
                        int(a_range[i] + (a_range[i + 1] - a_range[i]) * sig_val),
                    )

def is_current_platform_macos() -> bool:
    """
    Return True if the current process is running on macOS.
    Uses platform.system(), which should reliably return 'Darwin' on macOS.
    """
    return platform.system().lower() == "darwin"
