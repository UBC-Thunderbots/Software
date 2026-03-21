import threading
import time


class TimeProvider:
    """A singleton class that can provide time information"""

    def __init__(self) -> None:
        """Constructs time provider with initial timestamp at nanoseconds since epoch"""
        self._initial_timestamp = time.time_ns()
        self._current_timestamp = self._initial_timestamp
        self._timestamp_mutex = threading.Lock()

    def time_provider_ns(self) -> int:
        """... kind of a weird function"""
        with self._timestamp_mutex:
            return int(self._current_timestamp)

    def elapsed_time_ns(self) -> int:
        """Returns elapsed time since time provider was constructed in nanoseconds"""
        return int(self.time_provider_ns() - self._initial_timestamp)

    def tick_ns(self, delta_time_ns: float) -> None:
        """Advances time provider by a delta time in nanoseconds"""
        with self._timestamp_mutex:
            self._current_timestamp += delta_time_ns


time_provider_instance = TimeProvider()
