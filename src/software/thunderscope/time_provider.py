from abc import ABC, abstractmethod


class TimeProvider(ABC):
    """An interface for an object that can provide time information"""

    @abstractmethod
    def time_provider(self) -> float:
        """Provide the current time in seconds since the epoch"""
        raise NotImplementedError(
            "abstract method time_provider called from base class"
        )
