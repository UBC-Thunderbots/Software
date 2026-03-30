from dataclasses import dataclass
from enum import StrEnum, auto
from proto.import_all_protos import *
import software.python_bindings as tbots_cpp
from typing import Iterable


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


class Team(StrEnum):
    BLUE = auto()
    YELLOW = auto()


@dataclass
class BallState:
    position: tuple[float, float]
    velocity: tuple[float, float]


@dataclass
class RobotState:
    position: tuple[float, float]
    orientation: float
    velocity: tuple[float, float]
    angular_velocity: float


@dataclass
class TrackedEvent:
    event_type: EventType
    timestamp: float
    from_team: Team
    ball_state: BallState
    friendly_robot_states: list[RobotState]
    enemy_robot_states: list[RobotState]


def get_tuple_from_coords(coords: any) -> tuple[float, float]:
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
            position=get_tuple_from_coords(robot.position()),
            orientation=robot.orientiation().toRadians(),
            velocity=get_tuple_from_coords(robot.velocity()),
            angular_velocity=robot.angularVelocity().toRadians(),
        )
        for robot in team.getAllRobots()
    ]

def get_event_from_world(
    world: tbots_cpp.World, event_type: EventType, from_team: Team
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
        ball_state=ball_state,
        friendly_robot_states=friendly_states,
        enemy_robot_states=enemy_states,
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

    # Add friendly robots: [count, r1_data, r2_data...]
    row.append(str(len(event.friendly_robot_states)))
    for r in event.friendly_robot_states:
        row.extend(
            [
                r.position[0],
                r.position[1],
                r.orientation,
                r.velocity[0],
                r.velocity[1],
                r.angular_velocity,
            ]
        )

    # Add enemy robots: [count, r1_data, r2_data...]
    row.append(str(len(event.enemy_robot_states)))
    for r in event.enemy_robot_states:
        row.extend(
            [
                r.position[0],
                r.position[1],
                r.orientation,
                r.velocity[0],
                r.velocity[1],
                r.angular_velocity,
            ]
        )

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

    # Reconstruct BallState (x, y, vx, vy)
    ball_pos = (float(next(row_iter)), float(next(row_iter)))
    ball_vel = (float(next(row_iter)), float(next(row_iter)))
    ball_state = BallState(position=ball_pos, velocity=ball_vel)

    def parse_robots() -> list[RobotState]:
        # First, read the count marker
        count = int(next(row_iter))

        robots = []
        for _ in range(count):
            pos = (float(next(row_iter)), float(next(row_iter)))
            ori = float(next(row_iter))
            vel = (float(next(row_iter)), float(next(row_iter)))
            ang_vel = float(next(row_iter))
            robots.append(RobotState(pos, ori, vel, ang_vel))
        return robots

    friendly_robots = parse_robots()
    enemy_robots = parse_robots()

    return TrackedEvent(
        event_type=event_type,
        timestamp=timestamp,
        from_team=from_team,
        ball_state=ball_state,
        friendly_robot_states=friendly_robots,
        enemy_robot_states=enemy_robots,
    )
