from dataclasses import dataclass, field
from typing import List, Tuple, Any, override
from enum import Enum
import torch
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
    pass_speed: float

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
    # 1. Create the new edge index [2, 1]
    new_index = torch.tensor([[edge.from_index], [edge.to_index]], dtype=torch.long)

    # 2. Create the new features [1, num_features]
    edge_features = edge.get_edge_features()
    new_attr = torch.tensor([edge_features], dtype=torch.float)

    # 3. Check if this edge type already exists in data
    if edge.name in data.edge_types:
        # Concatenate along the edge dimension (dim=1 for index, dim=0 for attr)
        data[edge.name].edge_index = torch.cat(
            [data[edge.name].edge_index, new_index], dim=1
        )

        if len(edge_features) > 0:
            data[edge.name].edge_attr = torch.cat(
                [data[edge.name].edge_attr, new_attr], dim=0
            )
    else:
        # First time seeing this edge type
        data[edge.name].edge_index = new_index

        if len(edge_features) > 0:
            data[edge.name].edge_attr = new_attr

    return data
