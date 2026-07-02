import math
from abc import ABC, abstractmethod
from typing import Callable, Tuple, Optional, override
from dataclasses import dataclass
from proto.import_all_protos import *
from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtGui import *
from pyqtgraph.opengl import *
from software.py_constants import *
from software.thunderscope.gl.layers.gl_world_layer import GLWorldLayer
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.thunderscope.gl.sandbox.local_robot_state_provider import (
    local_robot_state_provider_instance,
)
from software.thunderscope.constants import Colors, DepthValues


class Operation(ABC):
    """Interface for operations that can be undone/redone"""

    @abstractmethod
    def reverse(self, friendly_next_id: int, enemy_next_id: int) -> "Operation":
        """Returns the reverse of this operation

        :param friendly_next_id: the next friendly robot id to use in the reversed operation
        :param enemy_next_id: the next enemy robot id to use in the reversed operation
        :return: the reverse of this operation
        """
        pass


@dataclass
class RobotOperation(Operation):
    """An operation that changes the state of the robots on the field
    Contains the id of the robot to change, its new and previous positions if applicable
    And the id of the next robot to add after this operation completes
    """

    id: int
    prev_pos: Optional[QVector3D]
    pos: Optional[QVector3D]
    next_id: int
    is_friendly: bool

    @override
    def reverse(self, friendly_next_id: int, enemy_next_id: int) -> Operation:
        return RobotOperation(
            id=self.id,
            prev_pos=self.pos,
            pos=self.prev_pos,
            next_id=friendly_next_id if self.is_friendly else enemy_next_id,
            is_friendly=self.is_friendly,
        )


@dataclass
class GroupOperation(Operation):
    """A group of RobotOperations that should be applied together"""

    operations: list[RobotOperation]

    @override
    def reverse(self, friendly_next_id: int, enemy_next_id: int) -> Operation:
        return GroupOperation(
            operations=[
                RobotOperation(
                    id=op.id,
                    prev_pos=op.pos,
                    pos=op.prev_pos,
                    next_id=friendly_next_id if op.is_friendly else enemy_next_id,
                    is_friendly=op.is_friendly,
                )
                for op in self.operations
            ]
        )


class EnemyAtMousePositionError(Exception):
    pass


class LastRobotRemoveError(Exception):
    pass


class GLSandboxWorldLayer(GLWorldLayer):
    """GLWorldLayer that adds functionality to add, remove, and change the state of the robots on the field"""

    class SandboxWorldState:
        """Wrapper around WorldState that automatically handles coordinate frame
        inversion when adding robots by delegating to the parent layer's
        _invert_position_if_defending_negative_half method.
        """

        def __init__(
            self,
            invert_fn: Callable[[QVector3D, float], Tuple[QVector3D, float]],
            friendly_colour_yellow: bool,
        ):
            self._proto = WorldState()
            self._invert_fn = invert_fn
            self._friendly_colour_yellow = friendly_colour_yellow

        @property
        def proto(self) -> WorldState:
            """Returns the underlying WorldState proto"""
            return self._proto

        def set_ball_state(self, ball_state: BallState) -> None:
            """Sets the ball state in the world state

            :param ball_state: the ball state to set
            """
            self._proto.ball_state.CopyFrom(ball_state)

        def add_robot(self, robot_id: int, pos: QVector3D, orientation: float) -> None:
            """Adds a robot to the world state, automatically inverting the
            position and orientation if the coordinate frame should be inverted

            :param robot_id: the id of the robot to add
            :param pos: the position of the robot
            :param orientation: the orientation of the robot (radians)
            """
            converted_pos, converted_orientation = self._invert_fn(
                QVector3D(pos.x(), pos.y(), 0), orientation
            )

            robot_state = RobotState(
                global_position=Point(
                    x_meters=converted_pos.x(),
                    y_meters=converted_pos.y(),
                ),
                global_orientation=Angle(radians=converted_orientation),
            )
            if self._friendly_colour_yellow:
                self._proto.yellow_robots[robot_id].CopyFrom(robot_state)
            else:
                self._proto.blue_robots[robot_id].CopyFrom(robot_state)

        def remove_robot(self, robot_id: int) -> None:
            """Removes a robot from the world state

            :param robot_id: the id of the robot to remove
            """
            if self._friendly_colour_yellow:
                del self._proto.yellow_robots[robot_id]
            else:
                del self._proto.blue_robots[robot_id]

    undo_toggle_enabled_signal = pyqtSignal(bool)
    redo_toggle_enabled_signal = pyqtSignal(bool)

    DEFAULT_ROBOT_ANGLE = 0

    MIN_ROBOT_ID = 0

    def __init__(
        self,
        name: str,
        simulator_io: ProtoUnixIO,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
    ):
        """Initializes a GLSandboxWorldLayer

        :param name: The displayed name of the layer
        :param simulator_io: The simulator io communicate with the simulator
        :param friendly_colour_yellow: Is the friendly_colour_yellow?
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary
        """
        super().__init__(name, simulator_io, friendly_colour_yellow, buffer_size)

        # double click flags for adding and removing
        self.robot_add_double_click = None
        self.robot_remove_double_click = None

        # the selected robot for moving
        self.selected_robot_id = None
        self.selected_robot_pos = None
        self.selected_robot_plane = None
        self.move_in_progress = False

        # the currently added robots and the next id to add
        self.friendly_next_id = 0
        self.enemy_next_id = 0
        
        self.friendly_curr_robots: set[int] = set()
        self.enemy_curr_robots: set[int] = set()
        self.curr_robot_ids_map: dict[bool, set[int]] = {
            True: self.friendly_curr_robots,
            False: self.enemy_curr_robots,
        }
        # if the world has robots already, update curr robot ids on the first tick
        self.should_init_curr_robot_ids = True

        # stacks for undo and redo operations
        self.undo_operations = []
        self.redo_operations = []

        self.sandbox_mode_enabled = False

        self._referee_defined = False

    @override
    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """Requires Ctrl + Shift to be pressed along with mouse click
        Gets the point(s) that the mouse click corresponds to on the xy-plane and other planes
        Determines if a robot is present at that position
        If so, sets fields to indicate that a robot is selected
        If not, checks for double click and adds a robot on the xy-plane coordinates

        :param event: the event containing the xy-plane and other plane coordinates
        """
        # forward event to super method for ball placement
        super().mouse_in_scene_pressed(event)

        # if sandbox mode is disabled, don't do anything
        if not self.sandbox_mode_enabled:
            return

        # only allow robot editing if Ctrl + Shift is pressed to avoid conflicting with the ball placement
        if not event.mouse_event.modifiers() & Qt.KeyboardModifier.ControlModifier:
            return

        # determine whether a robot was clicked
        robot_id, index, is_friendly = self.__identify_robot(event.multi_plane_points)

        # if an enemy robot is at the mouse position, return
        if robot_id is not None and not is_friendly:
            return

        if robot_id is None:
            # if no robot was clicked
            self.__handle_new_robot_event(event, is_friendly)
        else:
            # if a robot was clicked
            try:
                self.__handle_existing_robot_event(event, robot_id, index, is_friendly)
            except LastRobotRemoveError:
                # if the user attempted to remove the last robot
                # self.__display_last_remove_warning(event)
                return

        # robots are normally auto-rendered by refresh fn when sim is unpaused
        # when sim is paused, have to manually render
        if not self.is_playing:
            self._update_robots_graphics()

    @override
    def mouse_in_scene_dragged(self, event: MouseInSceneEvent) -> None:
        """Requires Ctrl + Shift to be pressed along with mouse click

        Gets the point(s) that the mouse has moved to on the xy-plane and other planes
        If a robot is currently selected, determines if another robot is present at the new position
        If so, returns
        If not, moves the selected robot to the new position

        :param event: the event containing the xy-plane and other plane coordinates
        """
        super().mouse_in_scene_dragged(event)

        # if sandbox mode is disabled, don't do anything
        if not self.sandbox_mode_enabled:
            return

        # only allow robot editing if Ctrl + Shift is pressed to avoid conflicting with the ball placement
        if not event.mouse_event.modifiers() & Qt.KeyboardModifier.ControlModifier:
            return

        # if robot is selected
        if self.selected_robot_id is not None and self.selected_robot_plane is not None:
            # get the new position on the plane that the robot was initially selected on
            point_on_current_plane = event.multi_plane_points[self.selected_robot_plane]

            # check if the mouse position is free or not
            robot_id, _, is_friendly = self.__identify_robot([point_on_current_plane])

            # skip if new position has a robot already
            if robot_id is not None:
                return

            if self.move_in_progress:
                # if a move is already in progress, update the undo move added previously with the new move position
                self.undo_operations[
                    len(self.undo_operations) - 1
                ].prev_pos = point_on_current_plane
            else:
                # add an undo operation to restore the robot to the position before moving
                self.__add_undo_operation(
                    RobotOperation(
                        self.selected_robot_id,
                        point_on_current_plane,
                        self.selected_robot_pos,
                        self.friendly_next_id,
                        is_friendly,
                    )
                )
                self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)
                # move is now in progress
                self.move_in_progress = True

            # update selected robot position
            self.__update_world_state(
                self.selected_robot_id,
                point_on_current_plane,
                self.DEFAULT_ROBOT_ANGLE,
                is_friendly=is_friendly,
            )

            # robots are normally auto-rendered by refresh fn when sim is unpaused
            # when sim is paused, have to manually render
            if not self.is_playing:
                self._update_robots_graphics()

    @override
    def mouse_in_scene_released(self, event: MouseInSceneEvent) -> None:
        """Reset the selected robot and the in progress move

        :param event: the mouse event
        """
        super().mouse_in_scene_released(event)

        # if sandbox mode is disabled, don't do anything
        if not self.sandbox_mode_enabled:
            return

        # ends the currently happening move
        self.selected_robot_id = None
        self.selected_robot_pos = None
        self.selected_robot_plane = None
        self.move_in_progress = False

    @override
    def _get_cached_teams_from_proto(self) -> None:
        super()._get_cached_teams_from_proto()

        self.__check_referee_status()

    @override
    def refresh_graphics(self) -> None:
        """Calls the super class refresh graphics

        If there are any pre-loaded robots in the world, updates curr robots ids
        to reflect this and stay consistent
        Ensures this is only done once
        """
        super().refresh_graphics()

        # if curr robot ids hasn't been synced yet
        if self.should_init_curr_robot_ids:
            # for robots in the world, add the ids them to curr robots
            for robot in self.cached_world.friendly_team.team_robots:
                self.friendly_curr_robots.add(robot.id)

            self.friendly_next_id = len(self.friendly_curr_robots)
            self.should_init_curr_robot_ids = False

    def __check_referee_status(self) -> None:
        """Checks if a referee message has been received.
        Once a referee message is received and has content, sets a flag
        permanently to True indicating we know which half we are defending.
        """
        referee = self.referee_buffer.get(block=False)
        if referee and referee.IsInitialized():
            self._referee_defined = True

    def undo(self) -> None:
        """Undoes the last operation
        Adds a corresponding opposite move to the redo list so we can redo if necessary
        """
        # skip if nothing to undo
        if len(self.undo_operations) == 0:
            return

        # get the operation which undoes the previous one
        operation = self.undo_operations.pop()

        # add the reverse operation to the redo list
        self.redo_operations.append(
            operation.reverse(self.friendly_next_id, self.enemy_next_id)
        )

        # apply the operation
        self.__undo_redo_internal(operation)

        # enable / disable the undo and redo buttons
        self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)
        self.redo_toggle_enabled_signal.emit(len(self.redo_operations) != 0)

        # robots are normally auto-rendered by refresh fn when sim is unpaused
        # when sim is paused, have to manually render
        if not self.is_playing:
            self._update_robots_graphics()

    def redo(self) -> None:
        """Redoes the last undo operation
        Adds a corresponding opposite move to the undo list so we can undo if necessary
        """
        # skip if nothing to redo
        if len(self.redo_operations) == 0:
            return

        # get the operation
        operation = self.redo_operations.pop()

        # add the reverse operation to the undo list
        self.undo_operations.append(
            operation.reverse(self.friendly_next_id, self.enemy_next_id)
        )

        # apply the operation
        self.__undo_redo_internal(operation)

        # enable / disable the undo and redo buttons
        self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)
        self.redo_toggle_enabled_signal.emit(len(self.redo_operations) != 0)

        # robots are normally auto-rendered by refresh fn when sim is unpaused
        # when sim is paused, have to manually render
        if not self.is_playing:
            self._update_robots_graphics()

    def clear_field(self) -> None:
        """Removes all robots from the field.
        Adds a GroupOperation to the undo list that can restore all robots.
        """
        # collect current robot positions into a GroupOperation for undo
        operations = {}

        # check the cached world state first
        for robot_ in self.cached_world.friendly_team.team_robots:
            # get the coordinates from the robot state
            pos_x = robot_.current_state.global_position.x_meters
            pos_y = robot_.current_state.global_position.y_meters

            operations[robot_.id] = RobotOperation(
                id=robot_.id,
                prev_pos=None,
                pos=QVector3D(
                    robot_.current_state.global_position.x_meters,
                    robot_.current_state.global_position.y_meters,
                    0,
                ),
                next_id=self.friendly_next_id,
                is_friendly=True,
            )

        # then, update with local state
        for robot_id, pos_and_orient in local_robot_state_provider_instance.get_team_state(self.friendly_colour_yellow).items():
            # if the local robot has already been removed, skip it
            if pos_and_orient is None:
                continue

            position, _ = pos_and_orient

            operations[robot_id] = RobotOperation(
                id=robot_id,
                prev_pos=None,
                pos=position,
                next_id=self.friendly_next_id,
                is_friendly=True,
            )

        if operations:
            self.undo_operations.append(GroupOperation(operations=operations.values()))
            self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)

        # clear internal state
        self.friendly_curr_robots.clear()
        self.enemy_curr_robots.clear()
        local_robot_state_provider_instance.clear_team(self.friendly_colour_yellow)
        local_robot_state_provider_instance.clear_team(not self.friendly_colour_yellow)

        # clear redo list since this is a new action
        self.redo_operations.clear()
        self.redo_toggle_enabled_signal.emit(False)

        # send out empty world state
        world_state = self.__get_empty_world_state()

        self.simulator_io.send_proto(WorldState, world_state.proto)

        # robots are normally auto-rendered by refresh fn when sim is unpaused
        # when sim is paused, have to manually render
        if not self.is_playing:
            self._update_robots_graphics()

    @override
    def toggle_play_state(self) -> bool:
        """When the simulator is paused / played, reset the local positions

        :return: the current boolean play state
        """
        # the super method handles the actual pausing of the simulator
        curr_play_state = super().toggle_play_state()

        # reset the local state
        local_robot_state_provider_instance.clear_team(self.friendly_colour_yellow)
        local_robot_state_provider_instance.clear_team(not self.friendly_colour_yellow)

        return curr_play_state

    def toggle_sandbox_mode(self) -> bool:
        """Toggles sandbox mode on/off, sends a SandboxModeState proto, and syncs undo/redo enable state

        :return: the current sandbox mode state
        """
        self.sandbox_mode_enabled = not self.sandbox_mode_enabled

        self.simulator_io.send_proto(
            SandboxModeState,
            SandboxModeState(is_enabled=self.sandbox_mode_enabled),
        )

        # resync undo / redo enabled state once sandbox mode is enabled
        # as enabling sandbox mode enables the buttons
        if self.sandbox_mode_enabled:
            self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)
            self.redo_toggle_enabled_signal.emit(len(self.redo_operations) != 0)

        return self.sandbox_mode_enabled

    def __add_undo_operation(self, operation: RobotOperation) -> None:
        """Adds an undo operation to the list and emits the toggle enable signal

        :param operation: the operation to add to the undo list
        """
        self.undo_operations.append(operation)
        self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)

    def __undo_redo_internal(self, operation: Operation) -> None:
        """Helper method to apply an Operation
        Updates robot positions and the next id

        :param operation: the operation to apply
        """
        if isinstance(operation, GroupOperation):
            self.friendly_next_id = float("inf")
            self.enemy_next_id = float("inf")

            world_state = self.__get_curr_world_state()

            for inner_op in operation.operations:
                if inner_op.is_friendly:
                    self.friendly_next_id = int(
                        min(self.friendly_next_id, inner_op.next_id)
                    )
                else:
                    self.enemy_next_id = int(
                        min(self.enemy_next_id, inner_op.next_id)
                    )
                world_state = self.__update_with_new_position(
                    world_state,
                    inner_op.id,
                    inner_op.pos,
                    self.DEFAULT_ROBOT_ANGLE,
                    inner_op.is_friendly,
                )

            # send out world state
            self.simulator_io.send_proto(WorldState, world_state.proto)
        else:
            if operation.is_friendly:
                self.friendly_next_id = operation.next_id
            else:
                self.enemy_next_id = operation.next_id
            self.__update_world_state(
                operation.id,
                operation.pos,
                self.DEFAULT_ROBOT_ANGLE,
                clear_redo=False,
                is_friendly=operation.is_friendly,
            )

    def __get_empty_world_state(self) -> SandboxWorldState:
        """Constructs a SandboxWorldState with just the ball state filled in
        from the cached world state and 1 robot placed at
        the edge of the center circle on the half line

        Replaces all local states as well

        :return: the sandbox world state with ball state and 1 robot
        """
        world_state = GLSandboxWorldLayer.SandboxWorldState(
            self.__invert_robot_if_defending_negative_half, self.friendly_colour_yellow
        )
        world_state.set_ball_state(self.cached_world.ball.current_state)

        center_circle_radius = self.cached_world.field.center_circle_radius
        robot_pos = QVector3D(-center_circle_radius, 0, 0)

        for robot_id in local_robot_state_provider_instance.get_team_state(self.friendly_colour_yellow).keys():
            local_robot_state_provider_instance.remove_robot(robot_id, self.friendly_colour_yellow)
        for robot_id in local_robot_state_provider_instance.get_team_state(not self.friendly_colour_yellow).keys():
            local_robot_state_provider_instance.remove_robot(robot_id, not self.friendly_colour_yellow)

        world_state = self.__update_with_new_position(
            world_state,
            self.MIN_ROBOT_ID,
            robot_pos,
            self.DEFAULT_ROBOT_ANGLE,
            is_friendly=True,
        )

        self.friendly_next_id = self.MIN_ROBOT_ID + 1

        return world_state

    # # # # # # # # # # # # # # # # # # # # # # # # #
    #       ADD / REMOVE / MOVE ROBOT METHODS       #
    # # # # # # # # # # # # # # # # # # # # # # # # #

    def __handle_existing_robot_event(
        self,
        event: MouseInSceneEvent,
        robot_id: int,
        index: int,
        is_friendly: bool,
    ) -> None:
        """Handles a mouse event when a position where a robot is present is clicked
        Marks the robot as selected (for drag moving)
        If double clicked, removes the robot at the position
        Else, starts a double click

        :param event: the event containing the xy-plane and other plane coordinates
        :param robot_id: the id of the robot that was clicked on
        :param index: the plane index that the robot was selected on
        :param is_friendly: whether the robot is on the friendly team
        """
        # marks the robot as selected along with the plane index that the mouse click intersected with
        # and its current position on that plane
        self.selected_robot_id = robot_id
        self.selected_robot_pos = event.multi_plane_points[index]
        self.selected_robot_plane = index

        # if double clicked
        if (
            self.robot_remove_double_click
            and self.robot_remove_double_click == event.multi_plane_points[index]
        ):
            # prevent removing the last robot
            if len(self.curr_robot_ids_map[is_friendly]) <= 1:
                self.__toggle_robot_remove_double_click()
                raise LastRobotRemoveError("Trying to remove the final robot!")

            # add an undo operation to add back the robot
            self.__add_undo_operation(
                RobotOperation(
                    robot_id,
                    None,
                    event.multi_plane_points[index],
                    self.friendly_next_id if is_friendly else self.enemy_next_id,
                    is_friendly,
                )
            )
            # remove the robot
            self.__update_world_state(
                robot_id,
                None,
                self.DEFAULT_ROBOT_ANGLE,
                is_friendly=is_friendly,
            )
            # set next id to the lowest free id
            if is_friendly:
                self.friendly_next_id = min(self.friendly_next_id, robot_id)
            else:
                self.enemy_next_id = min(self.enemy_next_id, robot_id)
            self.__toggle_robot_remove_double_click()
        else:
            # start a remove double click
            self.robot_remove_double_click = event.multi_plane_points[index]
            QTimer.singleShot(500, self.__toggle_robot_remove_double_click)

    def __display_last_remove_warning(self, event: MouseInSceneEvent) -> None:
        warning = GLTextItem(font=GLWorldLayer.TEXT_GRAPHICS_QFONT, color=Colors.RED)
        warning.show()
        warning.setDepthValue(DepthValues.ABOVE_FOREGROUND_DEPTH)
        warning.setData(
            text="Can't remove last robot!",
            pos=[
                event.position().x() - int(warning.width() / 2),
                event.position().y() + int(warning.height() * 1.1),
            ],
        )

    def __handle_new_robot_event(
        self, event: MouseInSceneEvent, is_friendly: bool
    ) -> None:
        """Handles a mouse event when an empty position is clicked
        If double clicked, adds a new robot at that position
        Else, starts a double click

        :param event: the mouse event with the new robot's position
        :param is_friendly: whether the robot is on the friendly team
        """
        # if the current point is a double click in progress
        if (
            self.robot_add_double_click
            and self.robot_add_double_click == event.point_in_scene
        ):
            curr_next_id = (
                self.friendly_next_id if is_friendly else self.enemy_next_id
            )

            # add an undo operation to remove the robot that is being added
            self.__add_undo_operation(
                RobotOperation(
                    curr_next_id,
                    event.point_in_scene,
                    None,
                    curr_next_id,
                    is_friendly,
                )
            )

            # add the robot
            self.__update_world_state(
                curr_next_id,
                event.point_in_scene,
                self.DEFAULT_ROBOT_ANGLE,
                is_friendly=is_friendly,
            )

            new_next_id = self.__get_next_robot_id(
                curr_next_id, self.curr_robot_ids_map[is_friendly]
            )

            if is_friendly:
                self.friendly_next_id = new_next_id
            else:
                self.enemy_next_id = new_next_id

            self.__toggle_robot_add_double_click()
        else:
            # start a double click
            self.robot_add_double_click = event.point_in_scene
            QTimer.singleShot(500, self.__toggle_robot_add_double_click)

    def __get_next_robot_id(self, curr_next_id: int, curr_robot_ids: set[int]) -> int:
        """Gets the id of the next robot to add based on the currently added robot ids

        :param curr_next_id: the current next id to add
        :param curr_robot_ids: the set of currently used robot ids for this team
        """
        # start with the default next id
        next_id = curr_next_id + 1

        # loops until a free id is found
        while next_id in curr_robot_ids:
            next_id += 1

        return next_id

    def __toggle_robot_add_double_click(self) -> None:
        """Resets the robot add double click flag"""
        if self.robot_add_double_click:
            self.robot_add_double_click = None

    def __toggle_robot_remove_double_click(self) -> None:
        """Resets the robot remove double click flag"""
        if self.robot_remove_double_click:
            self.robot_remove_double_click = None

    def __identify_robot(
        self, multi_plane_points: list[QVector3D]
    ) -> tuple[Optional[int], Optional[int]]:
        """Check local state first, then team robots, for a robot at the given position

        :param multi_plane_points: points on the x-y plane and planes above it
        :param team: the team to check (friendly or enemy)
        :param local_state_map: local robot position map (id -> (QVector3D, float) or None)
        :return: the robot id and plane index if found, else None, None
        """
        # check local state first
        for robot_id, pos in local_state_map.items():
            if pos is None:
                continue

            index = self.__identify_robot_with_pos(
                multi_plane_points, pos[0].x(), pos[0].y()
            )

            if index is not None:
                return robot_id, index

        # then check team robots
        for robot_ in team.team_robots:
            pos_x = robot_.current_state.global_position.x_meters
            pos_y = robot_.current_state.global_position.y_meters

            index = self.__identify_robot_with_pos(multi_plane_points, pos_x, pos_y)

            if index is not None:
                return robot_.id, index

        return None, None

    def __identify_robot(
        self, multi_plane_points: list[QVector3D]
    ) -> tuple[Optional[int], Optional[int], bool]:
        """Identify which robot was clicked on the field

        :param multi_plane_points: points on the x-y plane and planes above it corresponding to the mouse click
        :return: The robot id if one is present at the mouse position,
                 along with the index of the plane it was identified on,
                 and whether the robot is friendly (True) or enemy (False),
                 else None, None, False
        """
        # check friendly team first (with local state)
        robot_id, index = self.__identify_robot_with_locals(
            multi_plane_points,
            self.cached_world.friendly_team,
            self.friendly_local_robot_positions,
        )

        if robot_id is not None:
            return robot_id, index, True

        # check enemy team (with local state)
        robot_id, index = self.__identify_robot_with_locals(
            multi_plane_points,
            self.cached_world.enemy_team,
            self.enemy_local_robot_positions,
        )

        if robot_id is not None:
            return robot_id, index, False

        return None, None, False

    def __identify_robot_with_pos(
        self, multi_plane_points, pos_x, pos_y
    ) -> Optional[int]:
        """Loops over the multi plane points given and checks if any of them are within the radius of the given robot

        :param multi_plane_points: the points on the xy planes and planes above it to check
        :param pos_x: the x pos of the robot
        :param pos_y: the y pos of the robot
        :return: the index of the plane where the robot is identified
        """
        # consider points on the xy-plane, and a few planes above to account for robot height
        for index, point in enumerate(multi_plane_points):
            # if point is close enough to the robot, return the robot id and index it eas found on
            if (pos_x - point[0]) ** 2 + (pos_y - point[1]) ** 2 <= (
                ROBOT_MAX_RADIUS_MILLIMETERS / MILLIMETERS_PER_METER
            ) ** 2:
                return index
        return None

    def __get_curr_world_state(self) -> SandboxWorldState:
        world_state = GLSandboxWorldLayer.SandboxWorldState(
            self.__invert_robot_if_defending_negative_half, self.friendly_colour_yellow
        )

        # copy over existing robots for the current team
        for robot_ in self.cached_world.friendly_team.team_robots:
            world_state.add_robot(
                robot_.id,
                QVector3D(
                    robot_.current_state.global_position.x_meters,
                    robot_.current_state.global_position.y_meters,
                    0,
                ),
                robot_.current_state.global_orientation.radians,
                is_friendly=True,
            )

        # copy over any local state robots if sim is paused
        if not self.is_playing:
            for robot_id, pos in self.friendly_local_robot_positions.items():
                # if the robot has already been removed, skip it
                if pos is None:
                    continue

                world_state.add_robot(robot_id, pos[0], pos[1])

        return world_state

    def __update_world_state(
        self,
        new_robot_id: int,
        new_pos: Optional[QVector3D],
        new_orientation: float,
        clear_redo=True,
        is_friendly: bool = True,
    ) -> None:
        """Send out a WorldState proto with the existing robots
        If new position is provided, adds a robot with the given id at the given position
        Else, removes the robot with the given id from the robot state

        :param new_robot_id: the id of the robot to add / remove / move
        :param new_pos: the new QVector3D position of the robot (None if robot to be removed)
        :param new_orientation: the new orientation of the robot (radians)
        :param clear_redo: If True, indicates a new action instead of an action from the undo/redo list.
                            clears redo list if True
        :param is_friendly: whether the robot is on the friendly team
        """
        world_state = self.__get_curr_world_state()

        world_state = self.__update_with_new_position(
            world_state, new_robot_id, new_pos, new_orientation, is_friendly
        )

        # if we've just done a new action (not undone an old action)
        # we don't want to redo any past undone actions, so clear the redo list
        if clear_redo:
            self.redo_operations.clear()

        # send out world state
        self.simulator_io.send_proto(WorldState, world_state.proto)

    def __update_with_new_position(
        self,
        world_state: SandboxWorldState,
        robot_id: int,
        new_pos: Optional[QVector3D],
        new_orientation: float = 0,
        is_friendly: bool = True
    ) -> SandboxWorldState:
        """Updates the world state with the new robot position for the given id
        New position is defined if adding / moving a robot and None if removing one

        :param world_state: the world state to change
        :param robot_id: the id of thr robot to add / move / remove
        :param new_pos: the new position of the robot, or None if removing
        :param new_orientation: the new orientation of the robot if needed (radians)
        :param is_friendly: whether the robot is on the friendly team
        :return: the updated world state
        """
        if new_pos:
            self.curr_robot_ids_map[is_friendly].add(robot_id)
            world_state.add_robot(robot_id, new_pos, new_orientation)

            # saves the state to local dict
            self.local_robot_positions_map[is_friendly][robot_id] = (
                new_pos,
                new_orientation,
            )
        else:
            # remove an existing robot
            self.curr_robot_ids_map[is_friendly].remove(robot_id)
            self.local_robot_positions_map[is_friendly][robot_id] = None
            world_state.remove_robot(robot_id)

        return world_state

    @override
    def _should_invert_coordinate_frame(self) -> bool:
        if not self._referee_defined:
            return False

        return super()._should_invert_coordinate_frame()

    def __invert_robot_if_defending_negative_half(
        self, point: QVector3D, orientation: float
    ) -> tuple[QVector3D, float]:
        """If we are defending the negative half of the field, we invert the position and orientation
        of a robot  to match up with the visualization.

        :param point: The point location of the robot
        :param orientation: The orientation of the robot
        :return: The inverted location [x, y] and orientation (if needed to be inverted)

        """
        converted_point = self._invert_position_if_defending_negative_half(point)

        if self._should_invert_coordinate_frame():
            return converted_point, orientation - math.pi
        return converted_point, orientation

    # # # # # # # # # # # # # # # # # # # #
    #       GRAPHICS UPDATE METHODS       #
    # # # # # # # # # # # # # # # # # # # #

    def __override_cache_with_locals(
        self,
        local_positions: dict[int, tuple[QVector3D, float] | None],
        cached_team: dict[int, tuple[float, float, float]],
    ) -> None:
        """Override a cached team dict with local robot positions

        :param local_positions: map of robot id to (position, orientation) or None if removed
        :param cached_team: the cached team dict to override (e.g. self._cached_friendly_team)
        """
        for robot_id, pos in local_positions.items():
            if pos is None and robot_id in cached_team:
                del cached_team[robot_id]
            elif pos is not None:
                cached_team[robot_id] = (
                    pos[0].x(),
                    pos[0].y(),
                    pos[1],
                )

    @override
    def _update_robots_graphics(self) -> None:
        """Overrides the _update_robots_graphics method in the super class
        Adds local state robots to the team caches before updating the robot graphics
        """
        if not self.is_playing:
            self.__override_cache_with_locals(
                self.friendly_local_robot_positions, self._cached_friendly_team
            )
            self.__override_cache_with_locals(
                self.enemy_local_robot_positions, self._cached_enemy_team
            )

        super()._update_robots_graphics()
