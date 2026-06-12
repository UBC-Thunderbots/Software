from abc import ABC, abstractmethod


class ControllerBase(ABC):
    """Abstract base class for controller input sources."""

    @abstractmethod
    def name(self) -> str:
        """Get the display name of the input source."""
        ...

    @abstractmethod
    def connected(self) -> bool:
        """Return true if the input source is active and available."""
        ...

    @abstractmethod
    def key_down(self, key_code: int) -> bool:
        """Return true if the given key/button code is currently pressed."""
        ...

    @abstractmethod
    def abs_value(self, abs_code: int) -> float:
        """Return the current value of an axis, normalized to [-1, 1]."""
        ...

    @abstractmethod
    def close(self) -> None:
        """Release any resources held by the input source."""
        ...
