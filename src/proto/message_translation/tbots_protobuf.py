from __future__ import annotations

from proto.import_all_protos import *
import software.python_bindings as tbots_cpp
import numpy
import math


def create_world_state(
    yellow_robot_locations: list[tbots_cpp.Point],
    blue_robot_locations: list[tbots_cpp.Point],
    ball_location: tbots_cpp.Point,
    ball_velocity: tbots_cpp.Vector,
    blue_robot_orientations: list[tbots_cpp.Angle] = [],
    blue_robot_velocities: list[tbots_cpp.Vector] = [],
) -> WorldState:
    """Initializes the world from a list of robot locations and ball location/velocity.

    NOTE: (index is robot id)

    :param yellow_robot_locations: A list of yellow robot locations
    :param blue_robot_locations: A list of blue robot locations
    :param ball_location: Location of the ball
    :param ball_velocity: Velocity of the ball
    :param blue_robot_orientations: A list of blue robots orientations
    :param blue_robot_velocities: A list of blue robots velocities
    """
    world_state = WorldState()

    for robot_id, robot_location in enumerate(yellow_robot_locations):
        world_state.yellow_robots[robot_id].CopyFrom(
            RobotState(
                global_position=Point(
                    x_meters=robot_location.x(), y_meters=robot_location.y()
                ),
                global_orientation=Angle(radians=math.pi),
            )
        )

    for robot_id, robot_location in enumerate(blue_robot_locations):
        orientation = tbots_cpp.Angle.zero()
        velocity = tbots_cpp.Vector(0, 0)

        try:
            orientation = blue_robot_orientations[robot_id]
        except IndexError:
            pass

        try:
            velocity = blue_robot_velocities[robot_id]
        except IndexError:
            pass

        world_state.blue_robots[robot_id].CopyFrom(
            RobotState(
                global_position=Point(
                    x_meters=robot_location.x(), y_meters=robot_location.y()
                ),
                global_orientation=tbots_cpp.createAngleProto(orientation),
                global_velocity=tbots_cpp.createVectorProto(velocity),
            )
        )

    world_state.ball_state.CopyFrom(
        BallState(
            global_position=Point(
                x_meters=ball_location.x(), y_meters=ball_location.y()
            ),
            global_velocity=Vector(
                x_component_meters=ball_velocity.x(),
                y_component_meters=ball_velocity.y(),
            ),
        )
    )

    return world_state


def create_default_world_state(
    num_robots: int,
    field_x_length: float = 9.0,
    field_y_length: float = 6.0,
) -> WorldState:
    """Create a WorldState proto with num_robots yellow and blue robots evenly spaced in two parallel lines on the field.

    Robots are placed at one third of the field length from center on each side, and
    spread across two thirds of the field width, so the layout scales with field size.

    :param num_robots: Number of robots for the yellow and blue teams
    :param field_x_length: Field length along the x-axis in meters; defaults to SSL Division B
    :param field_y_length: Field width along the y-axis in meters; defaults to SSL Division B
    """
    x_offset = field_x_length / 3.0
    y_spread = field_y_length / 3.0
    return create_world_state(
        blue_robot_locations=[
            tbots_cpp.Point(-x_offset, y)
            for y in numpy.linspace(-y_spread, y_spread, num_robots)
        ],
        yellow_robot_locations=[
            tbots_cpp.Point(x_offset, y)
            for y in numpy.linspace(-y_spread, y_spread, num_robots)
        ],
        ball_location=tbots_cpp.Point(0, 0),
        ball_velocity=tbots_cpp.Vector(0, 0),
    )
