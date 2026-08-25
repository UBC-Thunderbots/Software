from typing import Callable

from pyqtgraph.Qt.QtGui import QVector3D


class LocalRobotStateProvider:
    """A singleton class that provides local robot state management

    Used to maintain the state of robots when the simulator is paused,
    so sandbox edits (add, remove, move robots) persist while paused.
    """

    def __init__(self) -> None:
        """Constructs a LocalRobotStateProvider with empty state"""
        # map of robot id to a tuple with the robot coordinates and orientation
        # or None if the robot has been removed already
        self.yellow_robot_states: dict[int, tuple[QVector3D, float] | None] = {}
        self.blue_robot_states: dict[int, tuple[QVector3D, float] | None] = {}
        self._callbacks: list[Callable[[], None]] = []

    def register_callback(self, callback: Callable[[], None]) -> None:
        """Register a callback to be invoked when local robot state changes

        :param callback: function to call when robot state changes
        """
        self._callbacks.append(callback)

    def _invoke_callbacks(self) -> None:
        """Invoke all registered callbacks"""
        for callback in self._callbacks:
            callback()

    def update_robot(
        self,
        robot_id: int,
        position: QVector3D,
        orientation: float,
        is_yellow: bool,
    ) -> None:
        """Add or update a robot's local state

        :param robot_id: the id of the robot
        :param position: the new position of the robot
        :param orientation: the new orientation of the robot (radians)
        :param is_yellow: whether the robot is on the yellow team
        """
        if is_yellow:
            self.yellow_robot_states[robot_id] = (position, orientation)
        else:
            self.blue_robot_states[robot_id] = (position, orientation)

        self._invoke_callbacks()

    def remove_robot(self, robot_id: int, is_yellow: bool) -> None:
        """Mark a robot as removed in local state

        :param robot_id: the id of the robot to remove
        :param is_yellow: whether the robot is on the yellow team
        """
        if is_yellow:
            self.yellow_robot_states[robot_id] = None
        else:
            self.blue_robot_states[robot_id] = None

        self._invoke_callbacks()

    def get_team_state(
        self, is_yellow: bool
    ) -> dict[int, tuple[QVector3D, float] | None]:
        """Get the full local state dict for a team

        :param is_yellow: whether to return the yellow team state
        :return: the dict of robot id to (position, orientation) or None
        """
        if is_yellow:
            return self.yellow_robot_states
        else:
            return self.blue_robot_states

    def clear_team(self, is_yellow: bool) -> None:
        """Clear all local state for a team

        :param is_yellow: whether to clear the yellow team state
        """
        if is_yellow:
            self.yellow_robot_states.clear()
        else:
            self.blue_robot_states.clear()

    def clear_all(self) -> None:
        """Clear all local robot state"""
        self.yellow_robot_states.clear()
        self.blue_robot_states.clear()


local_robot_state_provider_instance = LocalRobotStateProvider()
