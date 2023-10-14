import math
from typing import List, Tuple, Optional, Dict
from proto.import_all_protos import *
from pyqtgraph.Qt.QtCore import *
from pyqtgraph.Qt.QtGui import *
from pyqtgraph.opengl import *
from software.py_constants import *
from software.thunderscope.gl.layers.gl_world_layer import GLWorldLayer
from software.thunderscope.gl.graphics.gl_robot import GLRobot
from software.thunderscope.gl.helpers.extended_gl_view_widget import MouseInSceneEvent
from software.thunderscope.gl.helpers.observable_list import ObservableList
from software.thunderscope.constants import Colors


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


class GLSandboxWorldLayer(GLWorldLayer):
    """
    GLWorldLayer that adds functionality to add, remove, and change the state of the robots on the field
    """

    def __init__(
        self,
        name: str,
        simulator_io,
        friendly_colour_yellow: bool,
        buffer_size: int = 5,
    ):
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

        # the local state of robots (if simulator is paused)
        # map of robot id to a QVector3D object of the robot coordinates
        # or None if the robot has been removed already
        # (easier to keep track of robots rather than removing the entry entirely)
        self.local_robot_positions = {}

        # stacks for undo and redo operations
        self.undo_operations = []
        self.redo_operations = []

    def mouse_in_scene_pressed(self, event: MouseInSceneEvent):
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

        # determine whether a robot was clicked
        robot_id, index = self.__identify_robot(event.multi_plane_points)

        if robot_id is None:
            # if no robot was clicked
            self.__handle_new_robot_event(event)
        else:
            # if a robot was clicked
            self.__handle_existing_robot_event(event, robot_id, index)

    def mouse_in_scene_dragged(self, event: MouseInSceneEvent):
        """
        Requires Ctrl + Shift to be pressed along with mouse click

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
            robot_id, _ = self.__identify_robot([point_on_current_plane])

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
                self.undo_operations.append(
                    RobotOperation(
                        self.selected_robot_id,
                        point_on_current_plane,
                        self.selected_robot_pos,
                        self.next_id,
                    )
                )
                # move is now in progress
                self.move_in_progress = True

            # update selected robot position
            self.__update_world_state(self.selected_robot_id, point_on_current_plane)

    def mouse_in_scene_released(self, event: MouseInSceneEvent):
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

    def undo(self):
        """
        Undoes the last operation
        Adds a corresponding opposite move to the redo list so we can redo if necessary
        """
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
        # apply the operation
        self.__undo_redo_internal(operation)

    def redo(self):
        """
        Redoes the last undo operation
        Adds a corresponding opposite move to the undo list so we can undo if necessary
        """
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

    def __undo_redo_internal(self, operation: RobotOperation):
        """
        Helper method to apply a RobotOperation
        Updates robot positions and the next id
        :param operation: the operation to apply
        """
        self.next_id = operation.next_id
        self.__update_world_state(operation.id, operation.pos)

    # # # # # # # # # # # # # # # # # # # # # # # # #
    #       ADD / REMOVE / MOVE ROBOT METHODS       #
    # # # # # # # # # # # # # # # # # # # # # # # # #

    def __handle_existing_robot_event(
        self, event: MouseInSceneEvent, robot_id: int, index: int
    ):
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
            self.undo_operations.append(
                RobotOperation(
                    robot_id, None, event.multi_plane_points[index], self.next_id,
                )
            )
            # remove the robot
            self.__update_world_state(robot_id)
            # set next id to the lowest free id
            self.next_id = min(self.next_id, robot_id)
            self.__toggle_robot_remove_double_click()
        else:
            # start a remove double click
            self.robot_remove_double_click = event.multi_plane_points[index]
            QTimer.singleShot(500, self.__toggle_robot_remove_double_click)

    def __handle_new_robot_event(self, event: MouseInSceneEvent):
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
            self.undo_operations.append(
                RobotOperation(self.next_id, event.point_in_scene, None, self.next_id)
            )

            # add the robot
            self.__update_world_state(self.next_id, event.point_in_scene)
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

    def __toggle_robot_add_double_click(self):
        """
        Resets the robot add double click flag
        """
        if self.robot_add_double_click:
            self.robot_add_double_click = None

    def __toggle_robot_remove_double_click(self):
        """
        Resets the robot remove double click flag
        """
        if self.robot_remove_double_click:
            self.robot_remove_double_click = None

    def __add_robot_to_state(
        self, world_state: WorldState, id: int, robot_state: RobotState
    ):
        """
        Adds a robot with the given state and id to the given world state
        To the right team based on current team color
        :param world_state: the world state to add robot to
        :param id: the id of the robot to add
        :param robot_state: the state of the new robot to add
        """
        if self.friendly_colour_yellow:
            world_state.yellow_robots[id].CopyFrom(robot_state)
        else:
            world_state.blue_robots[id].CopyFrom(robot_state)
        return world_state

    def __remove_robot_from_state(self, world_state: WorldState, id: int):
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
        # first look for robot in local state
        for robot_id, pos in self.local_robot_positions.items():
            # if the local robot has already been removed, skip it
            if pos is None:
                continue

            # check if robot in position and return if found
            index = self.__identify_robot_helper(multi_plane_points, pos[0], pos[1])

            if index is not None:
                return robot_id, index

        # if not found, look for robot in cached world state
        # determine the current team
        team_robots, _ = self.__get_friendly_and_enemy_team()

        # look for robot in the cached world state
        for robot_ in team_robots:
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
            point_in_scene = self._invert_position_if_defending_negative_half(point)

            # if point is close enough to the robot, return the robot id and index it eas found on
            if (
                math.sqrt(
                    (pos_x - point_in_scene[0]) ** 2 + (pos_y - point_in_scene[1]) ** 2
                )
                <= ROBOT_MAX_RADIUS_MILLIMETERS / MILLIMETERS_PER_METER
            ):
                return index
        return None

    def __update_world_state(self, id: int, new_pos: Optional[QVector3D] = None):
        """
        Send out a WorldState proto with the existing robots
        If new position is provided, adds a robot with the given id at the given position
        Else, removes the robot with the given id from the robot state
        :param id: the id of the robot to add / remove
        :param new_pos: the position of the new robot [x, y] or None
        """
        world_state = WorldState()

        # determine current team based on current color
        team_robots, _ = self.__get_friendly_and_enemy_team()

        # copy over existing robots for the current team
        for robot_ in team_robots:
            world_state = self.__add_robot_to_state(
                world_state,
                robot_.id,
                RobotState(
                    global_position=Point(
                        x_meters=robot_.current_state.global_position.x_meters,
                        y_meters=robot_.current_state.global_position.y_meters,
                    ),
                ),
            )

        # copy over any local state robots if sim is paused
        if not self.is_playing:
            for robot_id, pos in self.local_robot_positions.items():
                # if the robot has already been removed, skip it
                if pos is None:
                    continue

                world_state = self.__add_robot_to_state(
                    world_state,
                    robot_id,
                    RobotState(
                        global_position=Point(x_meters=pos[0], y_meters=pos[1],),
                    ),
                )

            # if sim is paused, initialize the local state robot to the default value, overwritten later
            self.local_robot_positions[id] = None

        # update world state with new robot position
        if new_pos:
            # constructs and adds a new robot state
            self.curr_robot_ids.add(id)
            converted_pos = self._invert_position_if_defending_negative_half(new_pos)
            robot_state = RobotState(
                global_position=Point(
                    x_meters=converted_pos.x(), y_meters=converted_pos.y()
                ),
            )
            world_state = self.__add_robot_to_state(world_state, id, robot_state)
            if not self.is_playing:
                # update the local state to the converted position
                self.local_robot_positions[id] = (
                    converted_pos.x(),
                    converted_pos.y(),
                    0,
                )
        else:
            # remove an existing robot
            self.curr_robot_ids.remove(id)
            world_state = self.__remove_robot_from_state(world_state, id)

        # send out world state
        self.simulator_io.send_proto(WorldState, world_state)

    def __get_friendly_and_enemy_team(self) -> Tuple[List[Robot], List[Robot]]:
        """
        Gets the friendly and enemy team robots based on friendly_colour_yellow
        :return: A Tuple of [list of friendly robots, list of enemy robots]
        """
        if self.friendly_colour_yellow:
            return (
                self.cached_world.enemy_team.team_robots,
                self.cached_world.friendly_team.team_robots,
            )
        else:
            return (
                self.cached_world.friendly_team.team_robots,
                self.cached_world.enemy_team.team_robots,
            )

    def __get_friendly_and_enemy_color(self) -> Tuple[QColor, QColor]:
        """
        Gets the friendly and enemy colors based on friendly_colour_yellow
        :return: A Tuple of [friendly color, enemy color]
        """
        if self.friendly_colour_yellow:
            return Colors.YELLOW_ROBOT_COLOR, Colors.BLUE_ROBOT_COLOR
        else:
            return Colors.BLUE_ROBOT_COLOR, Colors.YELLOW_ROBOT_COLOR

    # # # # # # # # # # # # # # # # # # # #
    #       GRAPHICS UPDATE METHODS       #
    # # # # # # # # # # # # # # # # # # # #

    def _update_robots_graphics(self):
        """
        Called by the main refresh_graphics method in parent class
        Updates the robot graphics using the cached world state and the local robot state
        """
        # get the friendly / enemy teams and colors for this layer
        friendly_team, enemy_team = self.__get_friendly_and_enemy_team()
        friendly_color, enemy_color = self.__get_friendly_and_enemy_color()

        self.__update_robots_graphics(friendly_team, friendly_color, True)

        self.__update_robots_graphics(enemy_team, enemy_color, False)

    def __update_robots_graphics(
        self, team_robots: List[Robot], color: QColor, is_friendly: bool
    ):
        """
        Updates graphics for the given team with the given color
        If is_friendly, has to take into account both the robots in the world state and the local state
        Local state takes precedence over world state, so world state robots are overwritten when there are overlaps
        The resulting number of graphics are updated for the robots
        :param team_robots: a list of robot states to update graphics with
        :param color: the color of the robots
        :param is_friendly: if the current team is the friendly team for this layer
        """
        # start with the corresponding robot and robot id graphics before resizing
        robot_graphics = (
            self.friendly_robot_graphics if is_friendly else self.enemy_robot_graphics
        )
        robot_id_graphics = (
            self.friendly_robot_id_graphics
            if is_friendly
            else self.enemy_robot_id_graphics
        )

        # convert the team robots into a dictionary
        team_robots_dict = {
            robot.id: (
                robot.current_state.global_position.x_meters,
                robot.current_state.global_position.y_meters,
                robot.current_state.global_orientation.radians,
            )
            for robot in team_robots
        }

        # if this team is friendly, use local state to override any overlapping robots
        if is_friendly:
            for robot_id, pos in self.local_robot_positions.items():
                if pos is None and robot_id in team_robots_dict:
                    # if removed in local state, remove from dict
                    del team_robots_dict[robot_id]
                elif pos is not None:
                    # override position using local pos
                    team_robots_dict[robot_id] = pos

        # Ensure we have the same number of current graphics as world (+ local if friendly) robots
        robot_graphics.resize(len(team_robots_dict), lambda: GLRobot())
        robot_id_graphics.resize(
            len(team_robots_dict),
            lambda: GLTextItem(
                font=QFont("Roboto", 10, weight=700), color=Colors.PRIMARY_TEXT_COLOR,
            ),
        )

        # update graphics for the correct number of world robots
        self.__update_world_robot_graphics(
            team_robots_dict, color, robot_graphics, robot_id_graphics,
        )

    def __update_world_robot_graphics(
        self,
        robots: Dict[int, Tuple[int, int, int]],
        color: QColor,
        robot_graphics: ObservableList,
        robot_id_graphics: ObservableList,
    ):
        """
        Update the GLGraphicsItems with the robots from the cached world state

        :param robots: A dict of robot ids to a tuple containing [x_pos, y_pos, orientation] of the robot
        :param color: The color of the robots
        :param robot_graphics: The ObservableList containing the robot graphics for this team
        :param robot_id_graphics: The ObservableList containing the robot ID graphics for this team
        """

        # update the robot and robot id graphics for the filtered robots
        for robot_graphic, robot_id_graphic, robot_id in zip(
            robot_graphics, robot_id_graphics, robots,
        ):
            self._update_robot_graphic(
                robot_graphic,
                robot_id_graphic,
                color,
                robot_id,
                robots[robot_id][0:2],
                robots[robot_id][2],
            )
