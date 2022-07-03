

class DataLogger:

    """Checks if a ball enters any of the provided regions."""

    def __init__(self):
        pass

    def log_data(self, world, time_elapsed_s):
        raise NotImplementedError("log_data is not implemented")

    def get_data(self, world):
        raise NotImplementedError("log_data is not implemented")

