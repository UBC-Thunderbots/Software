from dataclasses import dataclass
from typing import Dict, Tuple, Any


@dataclass
class HeteroGNNConfig:
    ## Supports 2 options:
    # Mapping between (src, rel, dst) -> (GNN_Class, kwargs_for_that_layer)
    # Example: {('A', 'to', 'B'): (SAGEConv, {'aggr': 'max'})}
    # OR
    # A single convolution for all edges, eg: (SAGEConv, {'aggr': 'max'})
    edge_specs: (
        Tuple[Any, Dict[str, Any]]
        | Dict[Tuple[str, str, str], Tuple[Any, Dict[str, Any]]]
    )

    # Global architectural parameters

    # Number of features in the hidden layers
    hidden_channels: int

    # number of features for the final output layer
    out_channels: int

    # number of layers in total
    num_layers: int

    # Number of features in the input data
    # Can be -1 for lazy initialization
    in_channels: int = -1

    # How to merge different edge types at a target node (e.g., 'sum', 'mean', 'cat')
    global_aggr: str = "sum"

    return_node: str | None = None
