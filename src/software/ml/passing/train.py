import sys
import torch
import os
from torch_geometric.data import HeteroData
from torch_geometric.nn import SAGEConv, GATConv
from torch_geometric.loader import DataLoader
import torch.onnx
from typing import List
from software.ml.models.hetero_gnn.hetero_gnn import GenericHeteroGNN
from software.ml.models.hetero_gnn.config import HeteroGNNConfig
from software.ml.passing.data.types import NodeType
from software.ml.passing.data.build_graph import process_all_passes
from software.ml.passing.data.pass_result import generate_pass_results
from software.ml.passing.data.labelled_passes import LabelledPass, label_passes
from software.evaluation.logs.event_log import EventLog, Team
from software.evaluation.logs.pass_log import PassLog
import csv

dir_path = os.path.dirname(os.path.realpath(__file__))
onnx_path = os.path.join(dir_path, "onnx")


# --- 1. Data Hydration (CSV to Objects) ---
def load_and_label_data(
    pass_csv_file, event_csv_file, friendly_team
) -> List[LabelledPass]:
    pass_logs = []

    with open(pass_csv_file, mode="r", encoding="utf-8") as pass_csv:
        reader = csv.reader(pass_csv)

        for row in reader:
            pass_logs.append(PassLog.from_csv_row(iter(row)))

    event_logs = []
    with open(event_csv_file, mode="r", encoding="utf-8") as event_csv:
        reader = csv.reader(event_csv)

        for row in reader:
            event_logs.append(EventLog.from_csv_row(iter(row)))

    pass_results = generate_pass_results(
        event_logs=event_logs, pass_logs=pass_logs, friendly_team=friendly_team
    )

    labelled_passes = label_passes(pass_results)

    return labelled_passes


def train_single_model(
    dataset: HeteroData, model: GenericHeteroGNN, epochs=50, learning_rate=0.01
) -> GenericHeteroGNN:
    # 1. Setup the "Judge" (Loss Function) and the "Optimizer" (Weight Updater)
    # BCEWithLogitsLoss combines a Sigmoid layer and the BCELoss in one single class.
    # It's more numerically stable than using a plain Sigmoid followed by BCELoss.
    criterion = torch.nn.BCEWithLogitsLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    # 2. Use a DataLoader to handle batching multiple graphs together
    loader = DataLoader(dataset, batch_size=32, shuffle=True)

    model.train()
    for epoch in range(epochs):
        total_loss = 0
        for data in loader:
            optimizer.zero_grad()  # Clear old gradients from previous step

            # 3. Forward Pass: Get raw logits from the 'destination' node
            # logits shape: [batch_size, num_labels]
            logits = model(data.x_dict, data.edge_index_dict)

            # 4. Align the target labels
            # data.y was built from your Label dataclass
            target = data.y.view(logits.size())  # Ensure shapes match exactly

            # 5. Compute Loss
            loss = criterion(logits, target)

            # 6. Backward Pass: Calculate gradients
            loss.backward()

            # 7. Optimization: Nudge weights to reduce loss
            optimizer.step()

            total_loss += loss.item()

        print(f"Epoch {epoch+1}/{epochs} | Loss: {total_loss/len(loader):.4f}")

    return model


LAYER_OPTIONS = [2, 4]
CONVOLUTION_OPTIONS = [(SAGEConv, {"aggr": "mean"}), (GATConv, {"heads": 8})]
HIDDEN_DIMENSION_OPTIONS = [32, 64]

MODEL_NAME_TEMPLATE = "hetero_gnn_L{num_layers}_C{conv_type}_H{num_hidden_dims}"


def train_and_export_models(dataset: HeteroData):
    for num_layers in LAYER_OPTIONS:
        for convolution in CONVOLUTION_OPTIONS:
            for num_hidden_dims in HIDDEN_DIMENSION_OPTIONS:
                conv_name, args = convolution

                model_name = MODEL_NAME_TEMPLATE.format(
                    num_layers=num_layers,
                    conv_type=conv_name.__name__,
                    num_hidden_dims=num_hidden_dims,
                )

                metadata = dataset.metadata()
                num_labels = dataset[0].y.shape[-1]

                config = HeteroGNNConfig(
                    edge_specs=convolution,
                    num_layers=num_layers,
                    hidden_channels=num_hidden_dims,
                    out_channels=num_labels,
                    return_node=NodeType.PASS_DESTINATION.value,
                )

                model = GenericHeteroGNN(config=config, metadata=metadata)

                train_single_model(dataset=dataset, model=model)

                model.eval()

                x_dict = dataset[0].x_dict
                edge_dict = dataset[0].edge_index_dict

                file_name = os.path.join(onnx_path, f"{model_name}.onnx")

                torch.onnx.export(
                    model,
                    (x_dict, edge_dict),
                    file_name,
                    input_names=["x_dict", "edge_dict"],
                    output_names=["output"],
                    dynamic_axes={"x_dict": {0: "batch"}, "output": {0: "batch"}},
                    opset_version=15,  # Important for complex GNN ops
                )

                print(f"Saved {model_name}.onnx")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(
            "Usage: python train.py <pass_logs_csv_file_path> <event_logs_csv_file_path>"
        )

    labelled_passes = load_and_label_data(sys.argv[1], sys.argv[2], Team.BLUE)
    dataset = process_all_passes(labelled_passes=labelled_passes)
    train_and_export_models(dataset=dataset)
