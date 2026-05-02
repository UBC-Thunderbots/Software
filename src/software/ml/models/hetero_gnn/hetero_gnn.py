import torch
import torch.nn.functional as F
from torch_geometric.nn import HeteroConv
from software.ml.models.hetero_gnn.config import HeteroGNNConfig
from typing import override


class GenericHeteroGNN(torch.nn.Module):
    def __init__(self, config: HeteroGNNConfig, metadata: tuple):
        super().__init__()
        self.config = config
        self.convs = torch.nn.ModuleList()

        # Extract node_types and edge_types from metadata
        _, edge_types = metadata

        for i in range(config.num_layers):
            # Define input and output dims for this layer
            out_dim = (
                config.out_channels
                if i == config.num_layers - 1
                else config.hidden_channels
            )

            # Build the dictionary of convolutions for HeteroConv
            conv_dict = {}

            if isinstance(config.edge_specs, dict):
                # Specific mapping per edge type
                for edge_type, (layer_class, layer_kwargs) in config.edge_specs.items():
                    # We use (-1, -1) for in_channels to handle heterogeneous node feature sizes
                    conv_dict[edge_type] = layer_class(
                        (-1, -1), out_dim, **layer_kwargs
                    )
            else:
                # Single convolution for ALL edge types
                layer_class, layer_kwargs = config.edge_specs
                for edge_type in edge_types:
                    conv_dict[edge_type] = layer_class(
                        (-1, -1), out_dim, **layer_kwargs
                    )

            self.convs.append(HeteroConv(conv_dict, aggr=config.global_aggr))

        self.head_1s = torch.nn.Linear(config.hidden_channels, config.out_channels)
        self.head_5s = torch.nn.Linear(config.hidden_channels, config.out_channels)
        self.head_10s = torch.nn.Linear(config.hidden_channels, config.out_channels)

    @override
    def forward(self, x_dict, edge_index_dict):
        for i, conv in enumerate(self.convs):
            new_x_dict = conv(x_dict, edge_index_dict)

            for node_type, x in x_dict.items():
                if node_type not in new_x_dict:
                    new_x_dict[node_type] = x

            x_dict = new_x_dict

            # Apply activation/dropout except for the final layer
            if i < len(self.convs) - 1:
                x_dict = {key: F.relu(x) for key, x in x_dict.items()}
                x_dict = {
                    key: F.dropout(x, p=0.3, training=self.training)
                    for key, x in x_dict.items()
                }

        if self.config.return_node:
            return x_dict[self.config.return_node]
        else:
            return x_dict
