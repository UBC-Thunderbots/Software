import time
import collections
import statistics


class FrameTimeCounter:
    """FrameTimeCounter is basically just a list that stores the time difference
    between each consecutive function call.

    From that, it calculates the frametime of each function call.
    """

    def __init__(self) -> None:
        """Initialize FrameTimeCounter"""
        # stores the timeframe of every data cycle
        self.datapoints = collections.deque(maxlen=100)
        self.previous_timestamp = time.time()

    def add_one_datapoint(self) -> None:
        """Save the time difference between each consecutive function call."""
        current_time = time.time()
        time_difference = current_time - self.previous_timestamp
        self.datapoints.append(time_difference)
        self.previous_timestamp = current_time

    def get_last_frametime(self) -> float:
        """Return the latest frametime from the list

        :return: the latest frametime, and -1 if the list is empty
        """
        if not self.datapoints:
            return -1

        return self.datapoints[-1]

    def get_average_frametime(self) -> float:
        """Average the entire list

        :return: the average of the entire list, and -1 if the list is empty
        """
        if not self.datapoints:
            return -1

        return statistics.mean(self.datapoints)

    def get_average_last_30(self) -> float:
        """Average the last 30 frametime

        :return: the average of the last 30 frametime
        """
        if not self.datapoints:
            return -1

        return statistics.mean(list(self.datapoints)[-30:])
