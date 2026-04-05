from dataclasses import dataclass, field
from typing import List, Tuple, Any
from enum import Enum
from torch_geometric.data import HeteroData


class NodeType(Enum):
    ATTACKER = "attacker"
    RECEIVER = "receiver"
    ENEMY = "enemy"
    BALL = "ball"
    PASS_DESTINATION = "pass_destination"
    GLOBAL = "global_context"


@dataclass
class BaseEdge:
    name: Tuple[str, str, str] = field(init=False)
    from_index: int
    to_index: int

    def get_edge_features(self) -> list[Any]:
        return []


@dataclass
class GlobalToAnyEdge(BaseEdge):
    destination_name: NodeType

    def __post_init__(self):
        self.name = (NodeType.GLOBAL.value, "to", self.destination_name.value)


@dataclass
class AttackerBallEdge(BaseEdge):
    distance: float

    def __post_init__(self):
        self.name = (NodeType.ATTACKER.value, "to", NodeType.BALL.value)

    @override
    def get_edge_features(self) -> List[Any]:
        return [self.distance]


@dataclass
class BallPassDestinationEdge(BaseEdge):
    is_shot_on_goal: bool

    def __post_init__(self):
        self.name = (NodeType.BALL.value, "to", NodeType.PASS_DESTINATION.value)

    @override
    def get_edge_features(self) -> List[Any]:
        return [self.is_shot_on_goal]


@dataclass
class ReceiverPassDestinationEdge(BaseEdge):
    distance: float

    def __post_init__(self):
        self.name = (NodeType.RECEIVER.value, "to", NodeType.PASS_DESTINATION.value)

    @override
    def get_edge_features(self) -> List[Any]:
        return [self.distance]


@dataclass
class EnemyPassDestinationEdge(BaseEdge):
    distance: float
    intercept_dist: float
    time_delta: float

    def __post_init__(self):
        self.name = (NodeType.ENEMY.value, "to", NodeType.PASS_DESTINATION.value)

    @override
    def get_edge_features(self) -> List[Any]:
        return [self.distance, self.intercept_dist, self.time_delta]


def add_edge_to_data(edge: BaseEdge, data: HeteroData) -> HeteroData:
    data[edge.name].edge_index = (
        torch.tensor([edge.from_index, edge.to_index], dtype=torch.long)
        .t()
        .contiguous()
    )
    data[edge.name].edge_attr = torch.tensor(
        edge.get_edge_features(), dtype=torch.float
    )
