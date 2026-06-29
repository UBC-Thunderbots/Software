from pyqtgraph.Qt import QtGui
from pyqtgraph.opengl import *
from pyqtgraph.opengl.GLGraphicsItem import GLGraphicsItem

from software.py_constants import ROBOT_MAX_HEIGHT_METERS
from software.thunderscope.constants import Colors
from software.thunderscope.gl.graphics.gl_robot_outline import GLRobotOutline

from typing import Optional

import numpy as np


class GLRobot(GLMeshItem):
    """Displays a 3D mesh representing a robot"""

    def __init__(
        self,
        parent_item: Optional[GLGraphicsItem] = None,
        color: QtGui.QColor = Colors.DEFAULT_GRAPHICS_COLOR,
    ) -> None:
        """Initialize the GLRobot

        :param parent_item: The parent item of the graphic
        :param color: The color of the graphic
        """
        super().__init__(
            parentItem=parent_item,
            meshdata=self.__get_mesh_data(),
            color=color,
            smooth=False,
            shader="robotShader",
        )

        self.x = 0
        self.y = 0
        self.orientation = 0

    def set_position(self, x: float, y: float) -> None:
        """Set the position of the graphic in the scene

        :param x: The x coordinate to position the graphic at
        :param y: The y coordinate to position the graphic at
        """
        if self.x == x and self.y == y:
            return

        self.translate(x - self.x, y - self.y, 0)
        self.x = x
        self.y = y

    def set_orientation(self, degrees: float) -> None:
        """Set the orientation of the graphic in the scene

        :param degrees: The orientation of the graphic in degrees
        """
        # We need to add 45 degrees to our desired orientation in order
        # to get the flat side of the robot (i.e. its front) to face
        # the right way
        degrees += 45

        if self.orientation == degrees:
            return

        # Rotate locally about the z axis (0, 0, 1)
        self.rotate(degrees - self.orientation, 0, 0, 1, local=True)
        self.orientation = degrees

    def __get_mesh_data(self) -> MeshData:
        """Return a MeshData instance with vertices and faces computed
        for the surface of a cylinder with a flat side wall.
        This represents the geometry of a robot.

        :return: the computed MeshData instance
        """
        top_face_points = GLRobotOutline.get_robot_outline(
            z_coordinate=ROBOT_MAX_HEIGHT_METERS
        )
        bottom_face_points = GLRobotOutline.get_robot_outline(z_coordinate=0)
        circle_points = top_face_points + bottom_face_points

        # Add an extra point at the center of the top face. This is so that we
        # can construct the triangular faces that make up the top face by connecting
        # two adjacent points along the circle to the center point
        points = circle_points + [[0, 0, ROBOT_MAX_HEIGHT_METERS]]

        # Compute the triangular faces that make up the mesh.
        # Each face is an array of 3 indices in the points array.
        faces = []
        for index in range(len(top_face_points) - 1):
            faces.append([index, index + 1, index + len(top_face_points)])
        for index in range(len(top_face_points), len(circle_points) - 1):
            faces.append([index + 1 - len(top_face_points), index + 1, index])
        for index in range(len(top_face_points) - 1):
            faces.append([index, index + 1, len(circle_points)])

        return MeshData(
            vertexes=np.array(points),
            faces=np.array(faces),
        )


def init_robot_shader():
    """Adds a shader (robotShader) for rendering robots to the
    global list of shaders. The front of the robot is brightened
    to make it easier to see which way they're facing.
    """
    shaders.Shaders.append(
        shaders.ShaderProgram(
            "robotShader",
            [
                shaders.VertexShader("""
                varying vec3 normal;
                void main() {
                    // find vertex normals and positions
                    normal = normalize(gl_Normal);
                    gl_FrontColor = gl_Color;
                    gl_BackColor = gl_Color;
                    gl_Position = ftransform();
                }
            """),
                shaders.FragmentShader("""
                varying vec3 normal;
                void main() {
                    // create an alternate robot color (blue becomes teal, yellow becomes orange)
                    vec3 color = gl_Color.rgb;
                    vec3 color_bright = vec3(color.r, 0.65, color.b);
    
                    // this is the vector pointing forward from the robot
                    vec3 front = vec3(-1.0, 1.0, 0.0);
    
                    // mix colors depending on proximity of the face to the front of the robot
                    float fac = pow(max(dot(normal, front), 0.0), 0.5);
                    gl_FragColor = vec4(mix(color, color_bright, fac), 1.0);
                }
            """),
            ],
        )
    )


init_robot_shader()
