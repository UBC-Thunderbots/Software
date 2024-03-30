class TimeProvider:
    """An interface for an object that can provide time information"""

    def time_provider(self):
        """Provide the current time in seconds since the epoch"""
        raise NotImplementedError(
            "abstract method time_provider called from base class"
        )
