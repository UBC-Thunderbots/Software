from dataclasses import dataclass, fields
from enum import StrEnum, auto
from proto.import_all_protos import *
import software.python_bindings as tbots_cpp
from typing import Iterable, get_args, get_origin, Any
from software.py_constants import DIV_B_NUM_ROBOTS


class EventType(StrEnum):
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
    BLUE = auto()
    YELLOW = auto()


@dataclass
class BallState:
    position: tuple[float, float]
    velocity: tuple[float, float]


@dataclass
class RobotState:
    id: int
    position: tuple[float, float]
    orientation: float
    velocity: tuple[float, float]
    angular_velocity: float

    @classmethod
    def get_flattened_length(cls) -> int:
        count = 0
        for field in fields(cls):
            # Check if it's a tuple[float, float] etc.
            if get_origin(field.type) is tuple:
                count += len(get_args(field.type))
            else:
                count += 1
        return count


@dataclass
class TrackedEvent:
    event_type: EventType
    timestamp: float
    from_team: Team
    for_team: Team
    ball_state: BallState
    friendly_robot_states: list[RobotState]
    enemy_robot_states: list[RobotState]


def get_tuple_from_coords(coords: Any) -> tuple[float, float]:
    """Converts a coordinate object into a float tuple

    :param coords: the coordinate object (typically with .x() and .y() methods)
    :return: a tuple of (x, y) coordinates as floats
    """
    return (coords.x(), coords.y())


def get_robot_states_from_team(team: tbots_cpp.Team) -> list[RobotState]:
    """Extracts state data for all robots in a given team

    :param team: the team object containing robot data
    :return: a list of RobotState dataclasses for every robot on the team
    """
    return [
        RobotState(
            id=robot.id(),
            position=get_tuple_from_coords(robot.position()),
            orientation=robot.orientiation().toRadians(),
            velocity=get_tuple_from_coords(robot.velocity()),
            angular_velocity=robot.angularVelocity().toRadians(),
        )
        for robot in team.getAllRobots()
    ]


def get_event_from_world(
    world: tbots_cpp.World, event_type: EventType, from_team: Team, for_team: Team
) -> TrackedEvent:
    """Creates a TrackedEvent from a world object

    :param world_msg: the world object containing the state of the game
    :param event_type: the type of event being recorded
    :param from_team: the team that the event is coming from
    :return: a fully populated TrackedEvent including ball and robot states
    """
    ball = world.ball()
    ball_state = BallState(
        position=get_tuple_from_coords(ball.position()),
        velocity=get_tuple_from_coords(ball.velocity()),
    )

    friendly_states = get_robot_states_from_team(world.friendlyTeam())
    enemy_states = get_robot_states_from_team(world.enemyTeam())

    return TrackedEvent(
        timestamp=world.getMostRecentTimestamp().toSeconds(),
        event_type=event_type,
        from_team=from_team,
        for_team=for_team,
        ball_state=ball_state,
        friendly_robot_states=friendly_states,
        enemy_robot_states=enemy_states,
    )


def add_robot_state_to_row(row: list[Any], robot_states: list[RobotState]) -> None:
    """Serializes robot states into flattened columns within a list row

    :param row: the existing list representing a CSV row to be appended to
    :param robot_states: the list of RobotState objects to flatten and add
    :return: None (the row list is modified in place)
    """
    num_cols_per_robot = RobotState.get_flattened_length()

    # robot state columns will be added based on robot id
    robot_state_map = {robot_state.id: robot_state for robot_state in robot_states}

    # Add friendly robots: [r1_data, r2_data...] based on id
    for idx in range(DIV_B_NUM_ROBOTS):
        if idx not in robot_state_map:
            row.extend([None] * num_cols_per_robot)
        else:
            robot = robot_state_map[idx]
            row.extend(
                [
                    robot.position[0],
                    robot.position[1],
                    robot.orientation,
                    robot.velocity[0],
                    robot.velocity[1],
                    robot.angular_velocity,
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

    add_robot_state_to_row(row, event.friendly_robot_states)
    add_robot_state_to_row(row, event.enemy_robot_states)

    return ",".join(row)


def csv_row_to_event(row: Iterable[str]) -> TrackedEvent:
    """Parses a flat collection of CSV strings back into a TrackedEvent object

    :param row: an iterable of strings representing a single CSV row
    :return: a reconstructed TrackedEvent dataclass
    """
    # Convert strings to appropriate types
    # row[0] is EventType (StrEnum), row[1] is timestamp (float)
    row_iter = iter(row)

    event_type = EventType(next(row_iter))
    timestamp = float(next(row_iter))
    from_team = Team(next(row_iter))
    for_team = Team(next(row_iter))

    # Reconstruct BallState (x, y, vx, vy)
    ball_pos = (float(next(row_iter)), float(next(row_iter)))
    ball_vel = (float(next(row_iter)), float(next(row_iter)))
    ball_state = BallState(position=ball_pos, velocity=ball_vel)

    def parse_robots() -> list[RobotState]:
        """Parses flattened robot data from a CSV iterator, handling None entries.

        :param row_iter: an iterator over the CSV columns
        :return: a list of RobotState objects (skipping empty slots)
        """
        robots = []

        num_cols_per_robot = RobotState.get_flattened_length()

        for _ in range(DIV_B_NUM_ROBOTS):
            # Peek at the first field of the robot's data block
            raw_val = next(row_iter, None)

            # Check if the field is empty/null
            if raw_val in (None, "", "None"):
                # Consume the remaining columns for this "empty" robot slot
                # (Length - 1 because we already consumed the first column via 'raw_val')
                for _ in range(num_cols_per_robot - 1):
                    next(row_iter, None)
                continue

            try:
                # Reconstruct the RobotState now that we know data exists
                position = (float(raw_val), float(next(row_iter)))
                orientiation = float(next(row_iter))
                velocity = (float(next(row_iter)), float(next(row_iter)))
                angular_velocity = float(next(row_iter))

                robots.append(
                    RobotState(position, orientiation, velocity, angular_velocity)
                )
            except (ValueError, TypeError):
                # Handle cases where data might be malformed
                continue

        return robots

    friendly_robots = parse_robots()
    enemy_robots = parse_robots()

    return TrackedEvent(
        event_type=event_type,
        timestamp=timestamp,
        from_team=from_team,
        for_team=for_team,
        ball_state=ball_state,
        friendly_robot_states=friendly_robots,
        enemy_robot_states=enemy_robots,
    )
