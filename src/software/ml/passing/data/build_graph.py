import torch
import numpy as np
from torch_geometric.data import HeteroData
from typing import List
from dataclasses import fields
from software.evaluation.logs.pass_log import PassLogType
from software.ml.passing.data.labelled_passes import LabelledPass, Label
from software.ml.passing.data.types import (
    NodeType,
    AttackerBallEdge,
    BallPassDestinationEdge,
    ReceiverPassDestinationEdge,
    EnemyPassDestinationEdge,
    GlobalToAnyEdge,
    add_edge_to_data,
)


def get_dist(pos_1: List[float], pos_2: List[float]):
    return np.linalg.norm(np.array(pos_1) - np.array(pos_2))


def create_pyg_data(labelled_pass: LabelledPass) -> HeteroData:
    data = HeteroData()
    world_state = labelled_pass.pass_log.pass_event.world_state_log
    ball_state = world_state.ball_state

    ball_pos = np.array(ball_state.get_position())

    ## Building all Nodes

    # ball node
    data[NodeType.BALL.value].x = torch.tensor(
        [ball_state.to_array()], dtype=torch.float
    )

    # Attacker = Friendly robot closest to ball
    attacker_idx = min(
        range(len(world_state.friendly_robots)),
        key=lambda i: get_dist(world_state.friendly_robots[i].get_position(), ball_pos),
    )
    attacker = world_state.friendly_robots[attacker_idx]
    data[NodeType.ATTACKER.value].x = torch.tensor(
        [attacker.to_array()], dtype=torch.float
    )

    # Receivers - all other friendly robots
    receivers = [
        robot
        for i, robot in enumerate(world_state.friendly_robots)
        if i != attacker_idx
    ]
    data[NodeType.RECEIVER.value].x = torch.tensor(
        [receiver.to_array() for receiver in receivers], dtype=torch.float
    )

    # enemy robots as new node type
    enemies = world_state.enemy_robots
    data[NodeType.ENEMY.value].x = torch.tensor(
        [enemy_robot.to_array() for enemy_robot in enemies], dtype=torch.float
    )

    # pass destination as a new node
    pass_destination = labelled_pass.pass_log.get_pass_end_point()
    data[NodeType.PASS_DESTINATION.value].x = torch.tensor(
        [pass_destination], dtype=torch.float
    )

    # global stats as a node
    global_features = [
        x if x is not None else 0.0 for x in labelled_pass.result.to_array()
    ]
    data[NodeType.GLOBAL.value].x = torch.tensor([global_features], dtype=torch.float)

    attacker_ball_edge = AttackerBallEdge(
        distance=get_dist(attacker.get_position(), ball_pos), from_index=0, to_index=0
    )
    add_edge_to_data(attacker_ball_edge, data)

    ball_destination_edge = BallPassDestinationEdge(
        pass_speed=labelled_pass.pass_log.get_pass_speed(),
        is_shot_on_goal=False,
        from_index=0,
        to_index=0,
    )
    add_edge_to_data(ball_destination_edge, data)

    for index, receiver in enumerate(receivers):
        distance = get_dist(receiver.get_position(), pass_destination)
        edge = ReceiverPassDestinationEdge(
            distance=distance, from_index=index, to_index=0
        )
        add_edge_to_data(edge, data)

    for index, enemy in enumerate(enemies):
        distance = get_dist(enemy.get_position(), pass_destination)
        edge = EnemyPassDestinationEdge(
            distance=distance,
            intercept_dist=0,
            time_delta=0,
            from_index=index,
            to_index=0,
        )
        add_edge_to_data(edge, data)

    # add an edge from the global node to every other node
    for node_type in data.node_types:
        if node_type == NodeType.GLOBAL.value:
            continue

        num_target_nodes = data[node_type].num_nodes

        for idx in range(num_target_nodes):
            edge = GlobalToAnyEdge(
                from_index=0, to_index=idx, destination_name=NodeType(node_type)
            )
            add_edge_to_data(edge, data)

    ## Building Labels for data
    # Convert Label dataclass to one-hot vector
    label_array = []

    for label_type in PassLogType:
        if label_type == PassLogType.RESULT_0S:
            continue

        if label_type in labelled_pass.labels:
            label_dict = labelled_pass.labels[label_type].__dict__
            label_array.extend([int(v) for v in label_dict.values()])
        else:
            label_array.extend([0.0 for _ in range(len(fields(Label)))])

    label_vector = np.array(label_array, dtype=np.float32)

    data.y = torch.tensor(label_vector, dtype=torch.float)

    return data, label_vector


def process_all_passes(labelled_passes: List[LabelledPass]):
    graphs = []
    labels = []

    for labelled_pass in labelled_passes:
        graph, label = create_pyg_data(labelled_pass)
        graphs.append(graph)
        labels.append(label)

    return graphs, labels
