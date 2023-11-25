import math
from typing import List, Tuple, Optional, Dict
from proto.import_all_protos import *
from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtGui import *
from pyqtgraph.opengl import *
from software.py_constants import *
from software.thunderscope.gl.layers.gl_world_layer import GLWorldLayer
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent


class RobotOperation:
    """
    An operation that changes the state of the robots on the field
    Contains the id of the robot to change, its new and previous positions if applicable
    And the id of the next robot to add after this operation completes
    """

    def __init__(
        self,
        id: int,
        prev_pos: Optional[Tuple[int, int]],
        pos: Optional[Tuple[int, int]],
        next_id: int,
    ):
        self.type = type
        self.id = id
        self.prev_pos = prev_pos
        self.pos = pos
        self.next_id = next_id


class EnemyAtMousePositionError(Exception):
    pass


class GLSandboxWorldLayer(GLWorldLayer):
    """
    GLWorldLayer that adds functionality to add, remove, and change the state of the robots on the field
    """

    undo_toggle_enabled_signal = pyqtSignal(bool)
    redo_toggle_enabled_signal = pyqtSignal(bool)

    DEFAULT_ROBOT_ANGLE = 0

    def __init__(
        self,
        name: str,
        simulator_io,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
    ):
        """
        Initializes a GLSandboxWorldLayer

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
        self.next_id = 0
        self.curr_robot_ids = set()
        # if the world has robots already, update curr_robot_ids on the first tick
        self.should_init_curr_robot_ids = True

        # the local state of robots (if simulator is paused)
        # map of robot id to a Tuple with the robot coordinates and orientation
        # or None if the robot has been removed already
        # (easier to keep track of robots rather than removing the entry entirely)
        self.local_robot_positions: Dict[int, Tuple[QVector3D, float]] = {}

        # the state of robots before running the simulator
        # the robot state if only manual moves are considered
        self.pre_sim_robot_positions: Dict[int, Tuple[QVector3D, float]] = {}

        # stacks for undo and redo operations
        self.undo_operations = []
        self.redo_operations = []

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent) -> None:
        """
        Requires Ctrl + Shift to be pressed along with mouse click
        Gets the point(s) that the mouse click corresponds to on the xy-plane and other planes
        Determines if a robot is present at that position
        If so, sets fields to indicate that a robot is selected
        If not, checks for double click and adds a robot on the xy-plane coordinates
        :param event: the event containing the xy-plane and other plane coordinates
        """
        # forward event to super method for ball placement
        super().mouse_in_scene_pressed(event)

        # only allow robot editing if Ctrl + Shift is pressed to avoid conflicting with the ball placement
        if not event.mouse_event.modifiers() & Qt.KeyboardModifier.ControlModifier:
            return

        try:
            # determine whether a robot was clicked
            robot_id, index = self.__identify_robot(event.multi_plane_points)
        except EnemyAtMousePositionError:
            # if enemy robot is already at mouse position, return
            return

        if robot_id is None:
            # if no robot was clicked
            self.__handle_new_robot_event(event)
        else:
            # if a robot was clicked
            self.__handle_existing_robot_event(event, robot_id, index)

    def mouse_in_scene_dragged(self, event: MouseInSceneEvent) -> None:
        """
        Requires Ctrl + Shift to be pressed Ralong with mouse click

        Gets the point(s) that the mouse has moved to on the xy-plane and other planes
        If a robot is currently selected, determines if another robot is present at the new position
        If so, returns
        If not, moves the selected robot to the new position
        :param event: the event containing the xy-plane and other plane coordinates
        """
        super().mouse_in_scene_dragged(event)

        # only allow robot editing if Ctrl + Shift is pressed to avoid conflicting with the ball placement
        if not event.mouse_event.modifiers() & Qt.KeyboardModifier.ControlModifier:
            return

        # if robot is selected
        if self.selected_robot_id is not None and self.selected_robot_plane is not None:
            # get the new position on the plane that the robot was initially selected on
            point_on_current_plane = event.multi_plane_points[self.selected_robot_plane]

            # check if the mouse position is free or not
            try:
                robot_id, _ = self.__identify_robot([point_on_current_plane])
            except EnemyAtMousePositionError:
                # if enemy robot is already at mouse position, return
                return

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
                        self.next_id,
                    )
                )
                self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)
                # move is now in progress
                self.move_in_progress = True

            # update selected robot position
            self.__update_world_state(
                self.selected_robot_id, point_on_current_plane, self.DEFAULT_ROBOT_ANGLE
            )

    def mouse_in_scene_released(self, event: MouseInSceneEvent) -> None:
        """
        Reset the selected robot and the in progress move
        :param event: the mouse event
        """
        super().mouse_in_scene_released(event)

        # ends the currently happening move
        self.selected_robot_id = None
        self.selected_robot_pos = None
        self.selected_robot_plane = None
        self.move_in_progress = False

    def refresh_graphics(self) -> None:
        """
        Calls the super class refresh graphics

        If there are any pre-loaded robots in the world, updates curr robots ids
        to reflect this and stay consistent
        Ensures this is only done once
        """
        super().refresh_graphics()

        # if curr robot ids hasn't been synced yet
        if self.should_init_curr_robot_ids:
            # for robots in the world, add the ids them to curr robots
            for robot in self.cached_world.friendly_team.team_robots:
                self.curr_robot_ids.add(robot.id)
                self.pre_sim_robot_positions[robot.id] = (
                    QVector3D(
                        robot.current_state.global_position.x_meters,
                        robot.current_state.global_position.y_meters,
                        0,
                    ),
                    robot.current_state.global_orientation.radians,
                )

            self.next_id = len(self.curr_robot_ids)
            self.should_init_curr_robot_ids = False

    def undo(self) -> None:
        """
        Undoes the last operation
        Adds a corresponding opposite move to the redo list so we can redo if necessary
        """

        # skip if nothing to undo
        if len(self.undo_operations) == 0:
            return

        # get the operation which undoes the previous one
        operation = self.undo_operations.pop()

        # add an opposite operation to the redo list with pos and prev_pos swapped
        self.redo_operations.append(
            RobotOperation(
                id=operation.id,
                prev_pos=operation.pos,
                pos=operation.prev_pos,
                next_id=self.next_id,
            )
        )

        # enable / disable the undo and redo buttons
        self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)
        self.redo_toggle_enabled_signal.emit(len(self.redo_operations) != 0)

        # apply the operation
        self.__undo_redo_internal(operation)

    def redo(self) -> None:
        """
        Redoes the last undo operation
        Adds a corresponding opposite move to the undo list so we can undo if necessary
        """

        # skip if nothing to redo
        if len(self.redo_operations) == 0:
            return

        # get the operation
        operation = self.redo_operations.pop()

        # add an opposite operation to the undo list with pos and prev_pos swapped
        self.undo_operations.append(
            RobotOperation(
                id=operation.id,
                prev_pos=operation.pos,
                pos=operation.prev_pos,
                next_id=self.next_id,
            )
        )

        # enable / disable the undo and redo buttons
        self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)
        self.redo_toggle_enabled_signal.emit(len(self.redo_operations) != 0)

        # apply the operation
        self.__undo_redo_internal(operation)

    def toggle_play_state(self) -> bool:
        """
        When the simulator is paused / played, reset the local positions
        :return: the current boolean play state
        """
        # the super method handles the actual pausing of the simulator
        curr_play_state = super().toggle_play_state()

        # reset the local state
        self.local_robot_positions = {}

        return curr_play_state

    def reset_to_pre_sim(self) -> None:
        """
        Resets all robot positions to what they were before the simulator ran
        """
        for robot_id, state in self.pre_sim_robot_positions.items():
            self.__update_world_state(robot_id, state[0], state[1])

    def __add_undo_operation(self, operation: RobotOperation) -> None:
        """
        Adds an undo operation to the list and emits the toggle enable signal
        :param operation: the operation to add to the undo list
        """
        self.undo_operations.append(operation)
        self.undo_toggle_enabled_signal.emit(len(self.undo_operations) != 0)

    def __undo_redo_internal(self, operation: RobotOperation) -> None:
        """
        Helper method to apply a RobotOperation
        Updates robot positions and the next id
        :param operation: the operation to apply
        """
        self.next_id = operation.next_id
        self.__update_world_state(
            operation.id, operation.pos, self.DEFAULT_ROBOT_ANGLE, clear_redo=False
        )

    # # # # # # # # # # # # # # # # # # # # # # # # #
    #       ADD / REMOVE / MOVE ROBOT METHODS       #
    # # # # # # # # # # # # # # # # # # # # # # # # #

    def __handle_existing_robot_event(
        self, event: MouseInSceneEvent, robot_id: int, index: int
    ) -> None:
        """
        Handles a mouse event when a position where a robot is present is clicked
        Marks the robot as selected (for drag moving)
        If double clicked, removes the robot at the position
        Else, starts a double click
        :param event: the event containing the xy-plane and other plane coordinates
        :param robot_id: the id of the robot that was clicked on
        :param index: the plane index that the robot was selected on
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
            # add an undo operation to add back the robot
            self.__add_undo_operation(
                RobotOperation(
                    robot_id, None, event.multi_plane_points[index], self.next_id,
                )
            )
            # remove the robot
            self.__update_world_state(robot_id, None, self.DEFAULT_ROBOT_ANGLE)
            # set next id to the lowest free id
            self.next_id = min(self.next_id, robot_id)
            self.__toggle_robot_remove_double_click()
        else:
            # start a remove double click
            self.robot_remove_double_click = event.multi_plane_points[index]
            QTimer.singleShot(500, self.__toggle_robot_remove_double_click)

    def __handle_new_robot_event(self, event: MouseInSceneEvent) -> None:
        """
        Handles a mouse event when an empty position is clicked
        If double clicked, adds a new robot at that position
        Else, starts a double click
        :param event: the mouse event with the new robot's position
        """
        # if the current point is a double click in progress
        if (
            self.robot_add_double_click
            and self.robot_add_double_click == event.point_in_scene
        ):
            # add an undo operation to remove the robot that is being added
            self.__add_undo_operation(
                RobotOperation(self.next_id, event.point_in_scene, None, self.next_id)
            )

            # add the robot
            self.__update_world_state(
                self.next_id, event.point_in_scene, self.DEFAULT_ROBOT_ANGLE
            )
            self.next_id = self.__get_next_robot_id(self.next_id)
            self.__toggle_robot_add_double_click()
        else:
            # start a double click
            self.robot_add_double_click = event.point_in_scene
            QTimer.singleShot(500, self.__toggle_robot_add_double_click)

    def __get_next_robot_id(self, curr_next_id: int) -> int:
        """
        Gets the id of the next robot to add based on the currently added robot ids
        :param curr_next_id: the current next id to add
        """
        # start with the default next id
        next_id = curr_next_id + 1

        # loops until a free id is found
        while next_id in self.curr_robot_ids:
            next_id += 1

        return next_id

    def __toggle_robot_add_double_click(self) -> None:
        """
        Resets the robot add double click flag
        """
        if self.robot_add_double_click:
            self.robot_add_double_click = None

    def __toggle_robot_remove_double_click(self) -> None:
        """
        Resets the robot remove double click flag
        """
        if self.robot_remove_double_click:
            self.robot_remove_double_click = None

    def __add_robot_to_state(
        self, world_state: WorldState, id: int, pos: QVector3D, orientation: float
    ) -> WorldState:
        """
        Adds a robot with the given state and id to the given world state
        To the right team based on current team color
        Converts position and orientation if needed
        :param world_state: the world state to add robot to
        :param id: the id of the robot to add
        :param pos: the new QVector3D position of the robot
        :param orientation: the new orientation of the robot (radians)
        """
        # convert position and orientation if needed
        (
            converted_pos,
            converted_orientation,
        ) = self.__invert_robot_if_defending_negative_half(pos, orientation)
        # build the robot state
        robot_state = RobotState(
            global_position=Point(
                x_meters=converted_pos.x(), y_meters=converted_pos.y(),
            ),
            global_orientation=Angle(radians=converted_orientation),
        )

        if self.friendly_colour_yellow:
            world_state.yellow_robots[id].CopyFrom(robot_state)
        else:
            world_state.blue_robots[id].CopyFrom(robot_state)
        return world_state

    def __remove_robot_from_state(self, world_state: WorldState, id: int) -> WorldState:
        """
        Removes a robot with the given id from the right team in the given world state
        Based on current team color
        :param world_state: the world state to remove robot from
        :param id: the id of the robot to remove
        """
        if self.friendly_colour_yellow:
            del world_state.yellow_robots[id]
        else:
            del world_state.blue_robots[id]
        return world_state

    def __identify_robot(
        self, multi_plane_points: List[QVector3D]
    ) -> Tuple[Optional[int], Optional[int]]:
        """Identify which robot was clicked on the team

        :param multi_plane_points: points on the x-y plane and planes above it corresponding to the mouse click
        :returns: The robot if one is present at the mouse position,
                    along with the index of the plane it was identified on,
                    else None, None
        """
        # first, check if there's any enemy robots being clicked on
        # return None immediately if so (since we can't edit enemy robot state)
        for robot_ in self.cached_world.enemy_team.team_robots:
            # get the coordinates from the robot state
            pos_x = robot_.current_state.global_position.x_meters
            pos_y = robot_.current_state.global_position.y_meters

            # check if robot in position and return if found
            index = self.__identify_robot_helper(multi_plane_points, pos_x, pos_y)

            if index is not None:
                raise EnemyAtMousePositionError("Enemy robot at this position already!")

        # look for robot in local state
        for robot_id, pos in self.local_robot_positions.items():
            # if the local robot has already been removed, skip it
            if pos is None:
                continue

            # check if robot in position and return if found
            index = self.__identify_robot_helper(
                multi_plane_points, pos[0].x(), pos[0].y()
            )

            if index is not None:
                return robot_id, index

        # if not found, look for robot in cached world state
        for robot_ in self.cached_world.friendly_team.team_robots:
            # get the coordinates from the robot state
            pos_x = robot_.current_state.global_position.x_meters
            pos_y = robot_.current_state.global_position.y_meters

            # check if robot in position and return if found
            index = self.__identify_robot_helper(multi_plane_points, pos_x, pos_y)

            if index is not None:
                return robot_.id, index
        return None, None

    def __identify_robot_helper(
        self, multi_plane_points, pos_x, pos_y
    ) -> Optional[int]:
        """
        Loops over the multi plane points given and checks if any of them are within the radius of the given robot
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

    def __update_world_state(
        self,
        new_robot_id: int,
        new_pos: Optional[QVector3D],
        new_orientation: float,
        clear_redo=True,
    ) -> None:
        """
        Send out a WorldState proto with the existing robots
        If new position is provided, adds a robot with the given id at the given position
        Else, removes the robot with the given id from the robot state
        :param new_robot_id: the id of the robot to add / remove / move
        :param new_pos: the new QVector3D position of the robot (None if robot to be removed)
        :param new_orientation: the new orientation of the robot (radians)
        :param clear_redo: If True, indicates a new action instead of an action from the undo/redo list.
                            clears redo list if True
        """

        world_state = WorldState()

        # copy over existing robots for the current team
        for robot_ in self.cached_world.friendly_team.team_robots:
            world_state = self.__add_robot_to_state(
                world_state,
                robot_.id,
                QVector3D(
                    robot_.current_state.global_position.x_meters,
                    robot_.current_state.global_position.y_meters,
                    0,
                ),
                robot_.current_state.global_orientation.radians,
            )

        # copy over any local state robots if sim is paused
        if not self.is_playing:
            for robot_id, pos in self.local_robot_positions.items():
                # if the robot has already been removed, skip it
                if pos is None:
                    continue

                world_state = self.__add_robot_to_state(
                    world_state, robot_id, pos[0], pos[1]
                )

        world_state = self.__update_with_new_position(
            world_state, new_robot_id, new_pos, new_orientation
        )

        # if we've just done a new action (not undone an old action)
        # we don't want to redo any past undone actions, so clear the redo list
        if clear_redo:
            self.redo_operations.clear()

        # send out world state
        self.simulator_io.send_proto(WorldState, world_state)

    def __update_with_new_position(
        self,
        world_state: WorldState,
        robot_id: int,
        new_pos: Optional[QVector3D],
        new_orientation: float = 0,
    ) -> WorldState:
        """
        Updates the world state with the new robot position for the given id
        New position is defined if adding / moving a robot and None if removing one
        :param world_state: the world state to change
        :param robot_id: the id of thr robot to add / move / remove
        :param new_pos: the new position of the robot, or None if removing
        :param new_orientation: the new orientation of the robot if needed (radians)
        :return: the updated world state
        """
        if new_pos:
            self.curr_robot_ids.add(robot_id)
            world_state = self.__add_robot_to_state(
                world_state, robot_id, new_pos, new_orientation
            )

            # saves the state to local and pre-sim dicts
            self.pre_sim_robot_positions[robot_id] = (new_pos, new_orientation)
            if not self.is_playing:
                # update the local state to the converted position
                self.local_robot_positions[robot_id] = (
                    new_pos,
                    new_orientation,
                )
        else:
            # remove an existing robot
            self.curr_robot_ids.remove(robot_id)
            del self.pre_sim_robot_positions[robot_id]
            world_state = self.__remove_robot_from_state(world_state, robot_id)

        return world_state

    def __invert_robot_if_defending_negative_half(
        self, point: QVector3D, orientation: float
    ) -> Tuple[QVector3D, float]:
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

    def _update_robots_graphics(self) -> None:
        """
        Overrides the _update_robots_graphics method in the super class
        Adds local state robots to the friendly team cache before updating the robot graphics
        """
        for robot_id, pos in self.local_robot_positions.items():
            if pos is None and robot_id in self._cached_friendly_team:
                # if removed in local state, remove from dict
                del self._cached_friendly_team[robot_id]
            elif pos is not None:
                # override position using local pos
                self._cached_friendly_team[robot_id] = (pos[0].x(), pos[0].y(), pos[1])

        super()._update_robots_graphics()
