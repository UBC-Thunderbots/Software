from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.opengl import *

from proto.import_all_protos import *
from software.py_constants import *
import software.thunderscope.constants as constants
from software.thunderscope.constants import Colors

from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_rect import GLRect
from software.thunderscope.gl.graphics.gl_robot import GLRobot
from software.thunderscope.gl.graphics.gl_ball import GLBall

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from software.thunderscope.gl.gl_layer import GLLayer

class GLWorldLayer(GLLayer):

    def __init__(self, simulator_io, friendly_colour_yellow, buffer_size=5):
        GLLayer.__init__(self)

        self.simulator_io = simulator_io
        self.friendly_colour_yellow = friendly_colour_yellow

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.robot_status_buffer = ThreadSafeBuffer(buffer_size, RobotStatus)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee, False)
        self.cached_world = World()
        self.cached_status = {}

        self.key_pressed = {}
        self.accepted_keys = [Qt.Key.Key_Control, Qt.Key.Key_I, QtCore.Qt.Key.Key_Space]
        for key in self.accepted_keys:
            self.key_pressed[key] = False

        self.display_robot_id = False
        self.is_playing = True

        self.field_lines_rect = None
        self.field_centre_circle = None
        self.friendly_defense_area_rect = None
        self.enemy_defense_area_rect = None
        self.ball = None

        self.friendly_robots_by_id = {}
        self.enemy_robots_by_id = {}

        self.friendly_robot_id_text_items = {}
        self.enemy_robot_id_text_items = {}

    def keyPressEvent(self, event):
        """Detect when a key has been pressed

        NOTE: function name format is different due to overriding the Qt function

        :param event: The event

        """
        self.key_pressed[event.key()] = True
        if event.key() == QtCore.Qt.Key.Key_I:
            self.display_robot_id = not self.display_robot_id

        # If user is holding ctrl + space, send a command to simulator to pause the gameplay
        if (
            self.key_pressed[QtCore.Qt.Key.Key_Control]
            and self.key_pressed[QtCore.Qt.Key.Key_Space]
        ):

            simulator_state = SimulationState(is_playing=not self.is_playing)
            self.is_playing = not self.is_playing

            self.simulator_io.send_proto(SimulationState, simulator_state)

    def keyReleaseEvent(self, event):
        """Detect when a key has been released

        :param event: The event

        """
        self.key_pressed[event.key()] = False

    def updateFieldGraphics(self, field: Field):

        self.field_lines_rect.setDimensions(field.field_x_length, field.field_y_length)
        self.field_centre_circle.setRadius(field.center_circle_radius)

        self.friendly_defense_area_rect.setDimensions(field.defense_x_length, field.defense_y_length)
        self.friendly_defense_area_rect.setTranslation(
            -(field.field_x_length / 2) + (field.defense_x_length / 2),
            0
        )

        self.enemy_defense_area_rect.setDimensions(field.defense_x_length, field.defense_y_length)
        self.enemy_defense_area_rect.setTranslation(
            (field.field_x_length / 2) - (field.defense_x_length / 2),
            0
        )

    def updateBallGraphics(self, ball_state: BallState):

        self.ball.setPosition(
            ball_state.global_position.x_meters,
            ball_state.global_position.y_meters,
            ball_state.distance_from_ground
        )
    
    def updateRobotGraphics(self, team: Team, robot_id_map, robot_id_text_items, color):

        added_robot_graphics = []
        removed_robot_graphics = []

        for robot in team.team_robots:

            if robot.id not in robot_id_map:
                gl_robot = GLRobot(color)
                robot_id_map[robot.id] = gl_robot
                added_robot_graphics.append(gl_robot)

                robot_id_text_item = GLTextItem(text=str(robot.id))
                robot_id_text_items[robot.id] = robot_id_text_item
                added_robot_graphics.append(robot_id_text_item)

            gl_robot = robot_id_map[robot.id]
            gl_robot.setPosition(
                robot.current_state.global_position.x_meters,
                robot.current_state.global_position.y_meters 
            )

            robot_id_text_item = robot_id_text_items[robot.id]
            if self.display_robot_id:
                robot_id_text_item.show()
                robot_id_text_item.setData(
                    pos=[
                        robot.current_state.global_position.x_meters + (ROBOT_MAX_RADIUS_METERS / 2),
                        robot.current_state.global_position.y_meters,
                        ROBOT_MAX_HEIGHT_METERS + 0.1
                    ]
                )
            else:
                robot_id_text_item.hide()

        removed_robot_ids = [
            id for id in robot_id_map if id not in [
                robot.id for robot in team.team_robots
            ]
        ]
        for robot_id in removed_robot_ids:
            removed_robot_graphics.append(robot_id_map.pop(robot_id))
            removed_robot_graphics.append(robot_id_text_items.pop(robot_id))

        return added_robot_graphics, removed_robot_graphics
    
    def updateGraphics(self):
        
        added_graphics = []
        removed_graphics = []

        if not self.isVisible():
            
            if (self.field_lines_rect is not None):
                removed_graphics.append(self.field_lines_rect)
                self.field_lines_rect = None
            
            if (self.field_centre_circle is not None):
                removed_graphics.append(self.field_centre_circle)
                self.field_centre_circle = None
            
            if (self.friendly_defense_area_rect is not None):
                removed_graphics.append(self.friendly_defense_area_rect)
                self.friendly_defense_area_rect = None
            
            if (self.enemy_defense_area_rect is not None):
                removed_graphics.append(self.enemy_defense_area_rect)
                self.enemy_defense_area_rect = None
            
            if (self.ball is not None):
                removed_graphics.append(self.ball)
                self.ball = None

            for robot_id in list(self.friendly_robots_by_id):
                removed_graphics.append(self.friendly_robots_by_id.pop(robot_id))

            for robot_id in list(self.enemy_robots_by_id):
                removed_graphics.append(self.enemy_robots_by_id.pop(robot_id))

            for robot_id in list(self.friendly_robot_id_text_items):
                removed_graphics.append(self.friendly_robot_id_text_items.pop(robot_id))

            for robot_id in list(self.enemy_robot_id_text_items):
                removed_graphics.append(self.enemy_robot_id_text_items.pop(robot_id))

            return added_graphics, removed_graphics

        self.cached_world = self.world_buffer.get(block=False)
        
        if (self.field_lines_rect is None):
            self.field_lines_rect = GLRect(color=Colors.FIELD_LINE_COLOR)
            added_graphics.append(self.field_lines_rect)
        
        if (self.field_centre_circle is None):
            self.field_centre_circle = GLCircle(color=Colors.FIELD_LINE_COLOR)
            added_graphics.append(self.field_centre_circle)

        if (self.friendly_defense_area_rect is None):
            self.friendly_defense_area_rect = GLRect(color=Colors.FIELD_LINE_COLOR)
            added_graphics.append(self.friendly_defense_area_rect)

        if (self.enemy_defense_area_rect is None):
            self.enemy_defense_area_rect = GLRect(color=Colors.FIELD_LINE_COLOR)
            added_graphics.append(self.enemy_defense_area_rect)

        if (self.ball is None):
            self.ball = GLBall()
            added_graphics.append(self.ball)
        
        self.updateFieldGraphics(self.cached_world.field)
        self.updateBallGraphics(self.cached_world.ball.current_state)

        friendly_colour = (
            Colors.YELLOW_ROBOT_COLOR
            if self.friendly_colour_yellow
            else Colors.BLUE_ROBOT_COLOR
        )
        enemy_colour = (
            Colors.BLUE_ROBOT_COLOR
            if self.friendly_colour_yellow
            else Colors.YELLOW_ROBOT_COLOR
        )

        added_robot_graphics, removed_robot_graphics = self.updateRobotGraphics(
            self.cached_world.friendly_team, 
            self.friendly_robots_by_id,
            self.friendly_robot_id_text_items,
            friendly_colour
        )

        added_graphics += added_robot_graphics
        removed_graphics += removed_robot_graphics

        added_robot_graphics, removed_robot_graphics = self.updateRobotGraphics(
            self.cached_world.enemy_team, 
            self.enemy_robots_by_id,
            self.enemy_robot_id_text_items,
            enemy_colour)

        added_graphics += added_robot_graphics
        removed_graphics += removed_robot_graphics

        return added_graphics, removed_graphics

        