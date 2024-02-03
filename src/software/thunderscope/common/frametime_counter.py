import time
from PyQt6.QtWidgets import *

class FrameTimeCounter():
    """
    Frametimecounter is basically just a list that stores the time difference between each consecutaive function call.
    From that, it calcualtes the frametime of each function call.
    """

    def __init__(self) -> None:
        """
        Initialized framtime counter
        """
        self.datapoints = []  # stores the timeframe of every data cycle
        self.previous_timestamp = time.time()

    def add_one_datapoint(self):
        """
        Save the time difference between each consecutative function call.
        """
        current_time = time.time()
        time_difference = current_time - self.previous_timestamp
        self.datapoints.append(time_difference)

        self.previous_timestamp = current_time

    def get_last_frametime(self):
        """
        Return the latest frametime from the list

        :return: the latest frametime, and -1 if the list is empty
        """
        
        if len(self.datapoints) == 0:
            return -1

        return self.datapoints[-1]

    def get_average_frametime(self):
        """
        Averaget the entire list 

        :return: the average of the entire list, and -1 if the list is empty
        """
        if len(self.datapoints) == 0:
            return -1

        return sum(self.datapoints) / len(self.datapoints)

    def get_average_last_30(self):
        """
        Averaget the last 30 frametime

        :return: the average of the last 30 frametime
        """
        if (len(self.datapoints) == 0):
            return -1

        return sum(self.datapoints[-30:]) / 30

