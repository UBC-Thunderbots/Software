from dataclasses import dataclass
from enum import StrEnum, auto
from proto.import_all_protos import *
from typing import Any
from software.py_constants import DIV_B_NUM_ROBOTS
from google.protobuf.message import Message


class EventType(StrEnum):
    """Enum for the different types of events we want to track"""

    PASS = auto()
    SHOT_ON_GOAL = auto()
    ENEMY_SHOT_ON_GOAL = auto()
    SHOT_BLOCKED = auto()
    FRIENDLY_POSSESSION_START = auto()
    FRIENDLY_POSSESSION_END = auto()
    ENEMY_POSSESSION_START = auto()
    ENEMY_POSSESSION_END = auto()
    GAME_START = auto()
    GAME_END = auto()
    GOAL_SCORED = auto()
    YELLOW_CARD = auto()
    RED_CARD = auto()


class Team(StrEnum):
    """The teams present in the game"""

    BLUE = auto()
    YELLOW = auto()


def count_primitive_fields(message: Message):
    """Recursively counts the number of primitive fields in a Protobuf message."""
    count = 0
    # Use the descriptor to see all fields defined in the schema
    descriptor = message.DESCRIPTOR

    for field in descriptor.fields:
        # Check if the field is a nested message
        if field.type == FieldDescriptor.TYPE_MESSAGE:
            # Get the nested message class to recurse into its descriptor
            nested_message = field.message_type
            # Recurse using the nested message's descriptor
            count += count_leaf_fields_by_descriptor(nested_message)
        else:
            # It's a primitive type (double, float, int, bool, string, etc.)
            count += 1
    return count


@dataclass
class Robot:
    """Represents a single robot on the field, with ID and current state."""

    id: int
    state: RobotState


@dataclass
class TrackedEvent:
    """Represents a single event being tracked, where and for whom the event is, and the game state at the time of the event"""

    event_type: EventType
    timestamp: float
    from_team: Team
    for_team: Team
    ball_state: BallState
    friendly_robot_states: list[Robot]
    enemy_robot_states: list[Robot]


def get_event_from_world(
    world_msg: World, event_type: EventType, from_team: Team, for_team: Team
) -> TrackedEvent:
    """Creates a TrackedEvent from a world protobuf message

    :param world_msg: the world object containing the state of the game
    :param event_type: the type of event being recorded
    :param from_team: the team that the event is coming from
    :param for_team: the team that the event is for
    :return: a fully populated TrackedEvent including ball and robot states
    """
    ball_state = world_msg.ball.current_state

    friendly_states = [
        Robot(id=robot.id, state=robot.current_state)
        for robot in world_msg.friendly_team.team_robots
    ]
    enemy_states = [
        Robot(id=robot.id, state=robot.current_state)
        for robot in world_msg.enemy_team.team_robots
    ]

    return TrackedEvent(
        timestamp=world_msg.time_sent.epoch_timestamp_seconds,
        event_type=event_type,
        from_team=from_team,
        for_team=for_team,
        ball_state=ball_state,
        friendly_robot_states=friendly_states,
        enemy_robot_states=enemy_states,
    )


def add_robots_to_row(row: list[Any], robots: list[Robot]) -> None:
    """Serializes robots into flattened columns within a list row

    :param row: the existing list representing a CSV row to be appended to
    :param robot_states: the list of RobotState objects to flatten and add
    :return: None (the row list is modified in place)
    """
    num_cols_per_robot = count_primitive_fields(RobotState)

    # robot state columns will be added based on robot id
    robot_state_map = {robot.id: robot.state for robot in robots}

    # Add friendly robots: [r1_data, r2_data...] based on id
    for idx in range(DIV_B_NUM_ROBOTS):
        if idx not in robot_state_map:
            row.extend([None] * num_cols_per_robot)
        else:
            robot_state = robot_state_map[idx]
            row.extend(
                [
                    robot_state.global_position.x_meters,
                    robot_state.global_position.y_meters,
                    robot_state.global_orientation.radians,
                    robot_state.global_velocity.x_component_meters,
                    robot_state.global_velocity.y_component_meters,
                    robot_state.global_angular_velocity.radians_per_second,
                ]
            )


def event_to_csv_row(event: TrackedEvent) -> str:
    """Serializes a TrackedEvent into a flat CSV string row

    :param event: the TrackedEvent object to convert
    :return: a comma-separated string of all event attributes and robot states
    """
    row = [
        event.event_type.value,
        event.timestamp,
        *event.ball_state.position,
        *event.ball_state.velocity,
    ]

    add_robots_to_row(row, event.friendly_robots)
    add_robots_to_row(row, event.enemy_robots)

    return ",".join(row)
