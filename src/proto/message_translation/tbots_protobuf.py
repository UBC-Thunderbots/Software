from proto.import_all_protos import *
import software.python_bindings as cpp_bindings
import numpy


def create_world_state(
    yellow_robot_locations, blue_robot_locations, ball_location, ball_velocity
):
    """Initializes the world from a list of robot locations and ball location/velocity.

    NOTE: (index is robot id)

    :param yellow_robot_locations: A list of yellow robot locations
    :param blue_robot_locations: A list of blue robot locations
    :param ball_location: Location of the ball
    :param ball_velocity: Velocity of the ball

    """
    world_state = WorldState()

    for robot_id, robot_location in enumerate(yellow_robot_locations):
        world_state.yellow_robots[robot_id].CopyFrom(
            RobotState(
                global_position=Point(
                    x_meters=robot_location.x(), y_meters=robot_location.y()
                ),
            )
        )

    for robot_id, robot_location in enumerate(blue_robot_locations):
        world_state.blue_robots[robot_id].CopyFrom(
            RobotState(
                global_position=Point(
                    x_meters=robot_location.x(), y_meters=robot_location.y()
                ),
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


def create_default_world_state(num_robots):
    return create_world_state(
        blue_robot_locations=[
            cpp_bindings.Point(-3, y) for y in numpy.linspace(-2, 2, num_robots)
        ],
        yellow_robot_locations=[
            cpp_bindings.Point(3, y) for y in numpy.linspace(-2, 2, num_robots)
        ],
        ball_location=cpp_bindings.Point(0, 0),
        ball_velocity=cpp_bindings.Vector(0, 0),
    )
