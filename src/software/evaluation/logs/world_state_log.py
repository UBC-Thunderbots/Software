from __future__ import annotations
from dataclasses import dataclass
from proto.import_all_protos import *
from software.evaluation.logs.log_interface import IEvalLog, count_primitive_fields
from software.py_constants import DIV_B_NUM_ROBOTS
from typing import Any, Iterator, override


@dataclass
class RobotLog(IEvalLog):
    """Represents a single robot on the field, with ID and current state."""

    id: int
    state: RobotState

    num_cols: int = count_primitive_fields(RobotState.DESCRIPTOR)

    @classmethod
    @override
    def get_num_cols(cls) -> int:
        return RobotLog.num_cols

    def get_position(self) -> list[float]:
        """Returns the current ball position as a [float, float] array
        represnting x, y coordinates
        """
        return [
            self.state.global_position.x_meters,
            self.state.global_position.y_meters,
        ]

    @override
    def to_array(self) -> list[Any]:
        return self.get_position() + [
            self.state.global_orientation.radians,
            self.state.global_velocity.x_component_meters,
            self.state.global_velocity.y_component_meters,
            self.state.global_angular_velocity.radians_per_second,
        ]

    @staticmethod
    @override
    def from_csv_row(row_iter: Iterator[str], id: int = 0) -> RobotLog | None:
        data = [next(row_iter) for _ in range(RobotLog.num_cols)]

        # Check if this was a placeholder (all 'None' strings)
        if all(val == "None" for val in data):
            return None

        state = RobotState(
            global_position=Point(x_meters=float(data[0]), y_meters=float(data[1])),
            global_orientation=Angle(radians=float(data[2])),
            global_velocity=Vector(
                x_component_meters=float(data[3]), y_component_meters=float(data[4])
            ),
            global_angular_velocity=AngularVelocity(radians_per_second=float(data[5])),
        )

        # Since we don't store ID in the CSV row per your to_csv_row,
        # the ID is usually inferred by the caller based on index
        return RobotLog(id=id, state=state)


@dataclass
class BallLog(IEvalLog):
    """Represents a single ball on the field."""

    state: BallState

    num_cols: int = count_primitive_fields(BallState.DESCRIPTOR) - 1

    def get_position(self) -> list[float]:
        """Returns the current ball position as a [float, float] array
        represnting x, y coordinates
        """
        return [
            self.state.global_position.x_meters,
            self.state.global_position.y_meters,
        ]

    @classmethod
    @override
    def get_num_cols(cls) -> int:
        return BallLog.num_cols

    @override
    def to_array(self) -> list[Any]:
        return self.get_position() + [
            self.state.global_velocity.x_component_meters,
            self.state.global_velocity.y_component_meters,
        ]

    @staticmethod
    @override
    def from_csv_row(row_iter: Iterator[str]) -> BallLog | None:
        """Consumes columns from the iterator to reconstruct the Ball."""
        data = [next(row_iter) for _ in range(BallLog.num_cols)]

        state = BallState(
            global_position=Point(x_meters=float(data[0]), y_meters=float(data[1])),
            global_velocity=Vector(
                x_component_meters=float(data[2]), y_component_meters=float(data[3])
            ),
            distance_from_ground=0.0,
        )

        return BallLog(state=state)


@dataclass
class WorldStateLog(IEvalLog):
    """Represents the current state of the world"""

    ball_state: BallLog
    friendly_robots: list[RobotLog]
    enemy_robots: list[RobotLog]

    num_cols: int = (
        DIV_B_NUM_ROBOTS * RobotLog.get_num_cols() * 2 + BallLog.get_num_cols()
    )

    @staticmethod
    def from_world(world_msg: World) -> WorldStateLog:
        """Creates a WorldStateLog from a world protobuf message

        :param world_msg: the world object containing the state of the game
        :return: a fully populated WorldStateLog including ball and robot states
        """
        ball_state = BallLog(state=world_msg.ball.current_state)

        friendly_robots = [
            RobotLog(id=robot.id, state=robot.current_state)
            for robot in world_msg.friendly_team.team_robots
        ]
        enemy_robots = [
            RobotLog(id=robot.id, state=robot.current_state)
            for robot in world_msg.enemy_team.team_robots
        ]

        return WorldStateLog(
            ball_state=ball_state,
            friendly_robots=friendly_robots,
            enemy_robots=enemy_robots,
        )

    @classmethod
    @override
    def get_num_cols(cls) -> int:
        return WorldStateLog.num_cols

    def robots_to_array(self, robots: list[RobotLog]) -> list[Any]:
        """Serializes robots into flattened columns within a list

        :param robot_states: the list of RobotState objects to flatten
        :return: a list of values representing the states for all robots
        """
        num_cols_per_robot = RobotLog.get_num_cols()

        # robot state columns will be added based on robot id
        robot_state_map = {robot.id: robot for robot in robots}

        robots_array = []

        # Add friendly robots: [r1_data, r2_data...] based on id
        for idx in range(DIV_B_NUM_ROBOTS):
            new_state = []

            if idx not in robot_state_map:
                print(f"ROBOT {idx} NOT FOUND!!!!!!!!!!")
                new_state = [None for _ in range(num_cols_per_robot)]
            else:
                robot_state = robot_state_map[idx]
                new_state = robot_state.to_array()

            robots_array.extend(new_state)

        return robots_array

    @override
    def to_array(self) -> list[Any]:
        return (
            self.ball_state.to_array()
            + self.robots_to_array(self.friendly_robots)
            + self.robots_to_array(self.enemy_robots)
        )

    @staticmethod
    def from_csv_row(row_iter: Iterator[str]) -> WorldStateLog | None:
        # 1. Parse Ball
        ball_state = BallLog.from_csv_row(row_iter)

        # 2. Parse Friendly Robots
        friendly_robots = []
        for id in range(DIV_B_NUM_ROBOTS):
            robot = RobotLog.from_csv_row(row_iter, id=id)
            if robot:
                friendly_robots.append(robot)

        # 3. Parse Enemy Robots
        enemy_robots = []
        for id in range(DIV_B_NUM_ROBOTS):
            robot = RobotLog.from_csv_row(row_iter, id=id)
            if robot:
                enemy_robots.append(robot)

        return WorldStateLog(
            ball_state=ball_state,
            friendly_robots=friendly_robots,
            enemy_robots=enemy_robots,
        )
