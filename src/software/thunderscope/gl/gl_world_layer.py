from pyqtgraph.Qt import QtCore, QtGui
from pyqtgraph.Qt.QtCore import Qt
from pyqtgraph.opengl import *

from proto.import_all_protos import *
from software.py_constants import *
import software.python_bindings as geom
import software.thunderscope.constants as constants
from software.thunderscope.constants import Colors, SPEED_SEGMENT_SCALE

from software.thunderscope.gl.graphics.gl_circle import GLCircle
from software.thunderscope.gl.graphics.gl_rect import GLRect
from software.thunderscope.gl.graphics.gl_robot import GLRobot
from software.thunderscope.gl.graphics.gl_ball import GLBall

from software.networking.threaded_unix_listener import ThreadedUnixListener
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from software.thunderscope.gl.gl_layer import GLLayer


class GLWorldLayer(GLLayer):
    """GLLayer that visualizes the world and vision data"""

    # The maximum allowed velocity that the user can give the ball
    MAX_ALLOWED_KICK_SPEED_M_PER_S = 6.5

    def __init__(self, simulator_io, friendly_colour_yellow, buffer_size=5):
        """Initialize the GLWorldLayer

        :param simulator_io: The simulator io communicate with the simulator
        :param friendly_colour_yellow: Is the friendly_colour_yellow?
        :param buffer_size: The buffer size, set higher for smoother plots.
                            Set lower for more realtime plots. Default is arbitrary

        """
        GLLayer.__init__(self)

        self.simulator_io = simulator_io
        self.friendly_colour_yellow = friendly_colour_yellow

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)
        self.robot_status_buffer = ThreadSafeBuffer(buffer_size, RobotStatus)
        self.referee_buffer = ThreadSafeBuffer(buffer_size, Referee, False)
        self.cached_world = World()
        self.cached_status = {}

        self.key_pressed = {}
        self.accepted_keys = [
            Qt.Key.Key_Control,
            Qt.Key.Key_I,
            Qt.Key.Key_Space,
            Qt.Key.Key_Shift,
        ]
        for key in self.accepted_keys:
            self.key_pressed[key] = False

        self.display_robot_id = False
        self.display_speed_lines = True
        self.is_playing = True

        self.ball_velocity_vector = None
        self.point_in_scene_picked = None

        self.graphics_list.registerGraphicsGroup(
            "field_lines",
            lambda: GLRect(color=Colors.FIELD_LINE_COLOR)
        )
        self.graphics_list.registerGraphicsGroup(
            "field_center_circle",
            lambda: GLCircle(color=Colors.FIELD_LINE_COLOR)
        )
        self.graphics_list.registerGraphicsGroup("ball", GLBall)
        self.graphics_list.registerGraphicsGroup("robots", GLRobot)
        self.graphics_list.registerGraphicsGroup(
            "robot_ids", 
            lambda: GLTextItem(color=Colors.PRIMARY_TEXT_COLOR)
        )
        self.graphics_list.registerGraphicsGroup(
            "speed_lines",
            lambda: GLCircle(color=Colors.SPEED_VECTOR_COLOR)
        )

    def keyPressEvent(self, event):
        """Detect when a key has been pressed

        :param event: The event

        """
        self.key_pressed[event.key()] = True

        if event.key() == QtCore.Qt.Key.Key_I:
            self.display_robot_id = not self.display_robot_id
        elif event.key() == QtCore.Qt.Key.Key_S:
            self.display_speed_lines = not self.display_speed_lines

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

    def pointInScenePressed(self, event):
        """Event handler for the pointInScenePressed event
        
        :param event: The event
        
        """
        self.point_in_scene_picked = event.point_in_scene

        # Send a command to the simulator to move the ball to the picked point
        world_state = WorldState()
        world_state.ball_state.CopyFrom(
            BallState(
                global_position=Point(
                    x_meters=self.point_in_scene_picked[0],
                    y_meters=self.point_in_scene_picked[1],
                )
            )
        )
        self.simulator_io.send_proto(WorldState, world_state)

    def pointInSceneDragged(self, event):
        """Event handler for the pointInSceneDragged event
        
        :param event: The event
        
        """
        # User picked a point in the 3D scene and is now dragging it across the scene
        # to apply a velocity on the ball (i.e. kick it).
        # We create a velocity vector that is proportional to the distance the
        # mouse has moved away from the ball.

        ball_position = geom.Vector(
            self.point_in_scene_picked[0],
            self.point_in_scene_picked[1]
        )

        self.ball_velocity_vector = ball_position - geom.Vector(
            event.point_in_scene[0], event.point_in_scene[1]
        )

        # Cap the maximum kick speed
        if self.ball_velocity_vector.length() > GLWorldLayer.MAX_ALLOWED_KICK_SPEED_M_PER_S:
            self.ball_velocity_vector = self.ball_velocity_vector.normalize(
                GLWorldLayer.MAX_ALLOWED_KICK_SPEED_M_PER_S
            )

    def pointInSceneReleased(self, event):
        """Event handler for the pointInSceneReleased event
        
        :param event: The event
        
        """
        if self.ball_velocity_vector:

            # Send a command to the simulator to give the ball the specified 
            # velocity (i.e. kick it)

            world_state = WorldState()
            world_state.ball_state.CopyFrom(
                BallState(
                    global_position=Point(
                        x_meters=self.point_in_scene_picked[0],
                        y_meters=self.point_in_scene_picked[1],
                    ),
                    global_velocity=Vector(
                        x_component_meters=self.ball_velocity_vector.x(),
                        y_component_meters=self.ball_velocity_vector.y()
                    ),
                )
            )

            self.ball_velocity_vector = None
            self.simulator_io.send_proto(WorldState, world_state)

    def updateFieldGraphics(self, field: Field):
        """Update the GLGraphicsItems that display the field lines and markings
        
        :param field: The field proto

        """
        field_line_graphics = self.graphics_list.getGraphics("field_lines", 3)
        field_center_circle_graphic = self.graphics_list.getGraphics("field_center_circle", 1)[0]

        # Outer field lines
        field_line_graphics[0].setDimensions(field.field_x_length, field.field_y_length)
        
        # Center circle
        field_center_circle_graphic.setRadius(field.center_circle_radius)

        # Friendly defense area
        field_line_graphics[1].setDimensions(
            field.defense_x_length, field.defense_y_length
        )
        field_line_graphics[1].setPosition(
            -(field.field_x_length / 2) + (field.defense_x_length / 2), 0
        )

        # Enemy defense area
        field_line_graphics[2].setDimensions(
            field.defense_x_length, field.defense_y_length
        )
        field_line_graphics[2].setPosition(
            (field.field_x_length / 2) - (field.defense_x_length / 2), 0
        )

    def updateBallGraphics(self, ball_state: BallState):
        """Update the GLGraphicsItems that display the ball
        
        :param ball_state: The ball state proto

        """
        # Update the ball graphic
        ball_graphic = self.graphics_list.getGraphics("ball", 1)[0]
        ball_graphic.setPosition(
            ball_state.global_position.x_meters,
            ball_state.global_position.y_meters,
            ball_state.distance_from_ground,
        )

    def updateRobotGraphics(self, team: Team, color):
        """Update the GLGraphicsItems that display the robots
        
        :param team: The team proto
        :param color: The color of the robots

        """
        for robot_graphic, robot_id_graphic, robot in zip(
            self.graphics_list.getGraphics("robots", len(team.team_robots)), 
            self.graphics_list.getGraphics("robot_ids", len(team.team_robots)), 
            team.team_robots
        ):
            robot_graphic.setPosition(
                robot.current_state.global_position.x_meters,
                robot.current_state.global_position.y_meters,
            )
            robot_graphic.setOrientation(robot.current_state.global_orientation.radians)
            robot_graphic.setColor(color)

            if self.display_robot_id:
                robot_id_graphic.show()
                robot_id_graphic.setData(
                    text=str(robot.id),
                    pos=[
                        robot.current_state.global_position.x_meters
                        + (ROBOT_MAX_RADIUS_METERS / 2),
                        robot.current_state.global_position.y_meters,
                        ROBOT_MAX_HEIGHT_METERS + 0.1,
                    ]
                )
            else:
                robot_id_graphic.hide()

    def updateSpeedLineGraphics(self):
        """Update the speed lines visualizing the robot and ball speeds"""

        # If user if trying to apply a velocity on the ball (i.e. kick it), visualize
        # the velocity vector
        if self.ball_velocity_vector:
            
            ball_state = self.cached_world.ball.current_state
            velocity = self.ball_velocity_vector * SPEED_SEGMENT_SCALE

            speed_line_graphic = self.graphics_list.getGraphics("speed_lines", 1)[0]
            speed_line_graphic.setData(
                pos=np.array([
                    [
                        ball_state.global_position.x_meters, 
                        ball_state.global_position.y_meters
                    ],
                    [
                        ball_state.global_position.x_meters + velocity.x(), 
                        ball_state.global_position.y_meters + velocity.y()
                    ],
                ]),
            )

        if self.display_speed_lines:

            # Combining robots and ball into one list so we can avoid
            # multiple calls to GraphicsList.getGraphics
            objects = list(self.cached_world.friendly_team.team_robots)
            objects.append(self.cached_world.ball)
            
            for speed_line_graphic, object in zip(
                self.graphics_list.getGraphics("speed_lines", len(objects)),
                objects
            ):
                pos_x = object.current_state.global_position.x_meters
                pos_y = object.current_state.global_position.y_meters
                velocity = object.current_state.global_velocity
                speed_line_graphic.setData(
                    pos=np.array([
                        [pos_x, pos_y],
                        [
                            pos_x + velocity.x_component_meters * SPEED_SEGMENT_SCALE, 
                            pos_y + velocity.y_component_meters * SPEED_SEGMENT_SCALE
                        ],
                    ]),
                )

    def updateGraphics(self):
        """Update the GLGraphicsItems in this layer

        :returns: tuple (added_graphics, removed_graphics)
            - added_graphics - List of the added GLGraphicsItems
            - removed_graphics - List of the removed GLGraphicsItems
        
        """
        # Clear all graphics in this layer if not visible
        if not self.isVisible():
            return self.graphics_list.getChanges()

        self.cached_world = self.world_buffer.get(block=False)

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

        self.updateRobotGraphics(self.cached_world.friendly_team, friendly_colour)
        self.updateRobotGraphics(self.cached_world.enemy_team, enemy_colour)

        self.updateSpeedLineGraphics()

        return self.graphics_list.getChanges()
