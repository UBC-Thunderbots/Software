from proto.import_all_protos import *

def create_world_state(yellow_robot_locations, blue_robot_locations):
    """Initializes the world from a list of robot locations

    NOTE: (index is robot id)

    :param yellow_robot_locations: A list of yellowrobot locations
    :param blue_robot_locations: A list of blue robot locations

    """
    world_state = WorldState()

    for robot_id, robot_location in enumerate(yellow_robot_locations):
        world_state.yellow_robots[robot_id].CopyFrom(
                RobotState(
                    global_position=Point(
                        x_meters=robot_location.x(),
                        y_meters=robot_location.y()
                    ),
                )
            )

    for robot_id, robot_location in enumerate(blue_robot_locations):
        world_state.blue_robots[robot_id].CopyFrom(
                RobotState(
                    global_position=Point(
                        x_meters=robot_location.x(),
                        y_meters=robot_location.y()
                    ),
                )
            )

    return world_state

