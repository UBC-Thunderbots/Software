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
    def close(self) -> None:
        """Release any resources held by the input source."""
        ...

    def update(self) -> None:
        """Refresh controller input state. Called once per frame before reading inputs.
        Override for polled backends; push-based controllers can leave this as a no-op."""
        pass

    @abstractmethod
    def get_move_velocity(self) -> tuple[float, float, float]:
        """Return (x, y, angular) velocity, each normalized to [-1, 1] with deadzone applied.
        Positive x = forward, positive y = strafe left, positive angular = CCW."""
        ...

    @abstractmethod
    def get_speed_factor(self) -> float:
        """Return 1.0 normally, or SPEED_SLOWDOWN_FACTOR when slowdown input is active."""
        ...

    @abstractmethod
    def is_dribbler_held(self) -> bool:
        """Return True if the dribbler engage input is active."""
        ...

    @abstractmethod
    def get_kick_power_step(self) -> int:
        """Return -1, 0, or +1 for kick/chip power step direction.
        Non-zero only once per new input (edge-detected)."""
        ...

    @abstractmethod
    def get_dribbler_step(self) -> int:
        """Return -1, 0, or +1 for dribbler RPM step direction.
        Non-zero only once per new input (edge-detected)."""
        ...

    @abstractmethod
    def is_kick_fired(self) -> bool:
        """Return True once per kick button press (rising edge only)."""
        ...

    @abstractmethod
    def is_chip_fired(self) -> bool:
        """Return True once per chip button press (rising edge only)."""
        ...
