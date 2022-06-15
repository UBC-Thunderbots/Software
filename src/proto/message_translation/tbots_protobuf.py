from proto.import_all_protos import *
import software.python_bindings as tbots


def parse_world_state(world_state):
    yellow_robot_locations = [tbots.Point(robot.global_position.x_meters,robot.global_position.y_meters)  for robot in world_state.yellow_robots.values()]
    blue_robot_locations = [tbots.Point(robot.global_position.x_meters,robot.global_position.y_meters) for robot in world_state.blue_robots.values()]
    ball_location = tbots.Point(world_state.ball_state.global_position.x_meters, world_state.ball_state.global_position.y_meters)
    ball_velocity = tbots.Vector(world_state.ball_state.global_velocity.x_component_meters, world_state.ball_state.global_velocity.y_component_meters)

    return yellow_robot_locations, blue_robot_locations, ball_location, ball_velocity

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
