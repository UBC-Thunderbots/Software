class TimeProvider:
    """A singleton class that can provide time information"""

    def __init__(self) -> None:
        """Constructs time provider with no elapsed time"""
        self._elapsed_time = 0.0

    def elapsed_time_ns(self) -> float:
        """Returns elapsed time in nanoseconds since time provider was constructed"""
        return self._elapsed_time

    def tick_ns(self, delta_time_ns: float) -> None:
        """Advances elapsed time by a delta time in nanoseconds"""
        self._elapsed_time += delta_time_ns


time_provider_instance = TimeProvider()
