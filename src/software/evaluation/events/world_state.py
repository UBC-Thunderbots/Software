from dataclass import dataclass
from proto.import_all_protos import *
from software.evaluation.events.event import IEvent
from google.protobuf.descriptor import Descriptor, FieldDescriptor
from software.py_constants import DIV_B_NUM_ROBOTS

def count_primitive_fields(descriptor: Descriptor):
    """Recursively counts the number of primitive fields in a Protobuf message
    using its descriptor.

    :param message: the message descriptor to count all leaf-level primitive fields for
    :return: the count of primitive fields
    """
    count = 0

    for field in descriptor.fields:
        # Check if the field is a nested message
        if field.type == FieldDescriptor.TYPE_MESSAGE:
            # Get the nested message class to recurse into its descriptor
            nested_message = field.message_type
            # Recurse using the nested message's descriptor
            count += count_primitive_fields(nested_message)
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
class WorldState(IEvent):  
  timestamp: float
  ball_state: BallState
  friendly_robots: list[Robot]
  enemy_robots: list[Robot]
  
  @staticmethod
  def from_world(
      world_msg: World
  ) -> WorldState:
      """Creates a TrackedEvent from a world protobuf message

      :param world_msg: the world object containing the state of the game
      :param event_type: the type of event being recorded
      :param from_team: the team that the event is coming from
      :param for_team: the team that the event is for
      :return: a fully populated TrackedEvent including ball and robot states
      """
      ball_state = world_msg.ball.current_state

      friendly_robots = [
          Robot(id=robot.id, state=robot.current_state)
          for robot in world_msg.friendly_team.team_robots
      ]
      enemy_robots = [
          Robot(id=robot.id, state=robot.current_state)
          for robot in world_msg.enemy_team.team_robots
      ]

      return WorldState(
          timestamp=world_msg.time_sent.epoch_timestamp_seconds,
          ball_state=ball_state,
          friendly_robots=friendly_robots,
          enemy_robots=enemy_robots,
      )
      
  @override
  def get_timestamp(self) -> float:
      return self.timestamp
      
  def add_robots_to_row(self, row: list[Any], robots: list[Robot]) -> None:
    """Serializes robots into flattened columns within a list row

    :param row: the existing list representing a CSV row to be appended to
    :param robot_states: the list of RobotState objects to flatten and add
    :return: None (the row list is modified in place)
    """
    num_cols_per_robot = count_primitive_fields(RobotState.DESCRIPTOR)

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
    
  @override
  def to_csv_row(self):
    """Serializes a TrackedEvent into a flat CSV string row

    :param event: the TrackedEvent object to convert
    :return: a comma-separated string of all event attributes and robot states
    """
    row = [self.timestamp]

    row = row + [
        self.ball_state.global_position.x_meters,
        self.ball_state.global_position.y_meters,
        self.ball_state.global_velocity.x_component_meters,
        self.ball_state.global_velocity.y_component_meters,
    ]

    self.add_robots_to_row(row, self.friendly_robots)
    self.add_robots_to_row(row, self.enemy_robots)

    return ",".join([str(elem) for elem in row])
  