import random
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
from software.evaluation.logs.pass_log import PassLog, PassLogType
import csv

dir_path = os.path.dirname(os.path.realpath(__file__))
ml_dir_path = os.path.dirname(dir_path)
datasets_path = os.path.join(ml_dir_path, "datasets")
onnx_path = os.path.join(dir_path, "onnx")


# --- 1. Data Hydration (CSV to Objects) ---
def load_and_label_data(
    pass_csv_file, event_csv_file, friendly_team
) -> List[LabelledPass]:
    pass_logs = []

    with open(
        os.path.join(datasets_path, pass_csv_file), mode="r", encoding="utf-8"
    ) as pass_csv:
        reader = csv.reader(pass_csv)

        for row in reader:
            pass_logs.append(PassLog.from_csv_row(iter(row)))

    print("Passes loaded!")

    event_logs = []
    with open(
        os.path.join(datasets_path, event_csv_file), mode="r", encoding="utf-8"
    ) as event_csv:
        reader = csv.reader(event_csv)

        for row in reader:
            event_logs.append(EventLog.from_csv_row(iter(row)))

    print("Events loaded!")

    pass_results = generate_pass_results(
        event_logs=event_logs, pass_logs=pass_logs, friendly_team=friendly_team
    )

    print("Pass Results Generated!")

    labelled_passes = label_passes(pass_results)

    print("Passes labelled!")

    return labelled_passes


def train_single_model(
    loader: DataLoader, model: GenericHeteroGNN, epochs=50, learning_rate=0.01
) -> GenericHeteroGNN:
    # 1. Setup the "Judge" (Loss Function) and the "Optimizer" (Weight Updater)
    # BCEWithLogitsLoss combines a Sigmoid layer and the BCELoss in one single class.
    # It's more numerically stable than using a plain Sigmoid followed by BCELoss.
    criterion = torch.nn.BCEWithLogitsLoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    model.train()

    for epoch in range(epochs):
        total_loss = 0
        for data in loader:
            optimizer.zero_grad()  # Clear old gradients from previous step

            # 3. Forward Pass: Get raw logits from the 'destination' node
            # logits shape: [batch_size, num_types, num_labels]
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

TRAIN_DATA_CROSS_RATIO = 0.8


def evaluate_model(loader: DataLoader, model: GenericHeteroGNN):
    criterion = torch.nn.BCEWithLogitsLoss()
    model.eval()

    intervals = [
        interval for interval in PassLogType if interval != PassLogType.RESULT_0S
    ]

    stats = {
        interval: {"total_loss": 0.0, "correct_bits": 0.0, "count": 0}
        for interval in intervals
    }

    with torch.no_grad():
        for data in loader:
            # 1. Forward pass
            logits = model(data.x_dict, data.edge_index_dict)
            target = data.y.view(logits.size())

            logits_len = len(logits[0])
            logits_per_interval = logits_len // len(intervals)

            # 2. Iterate through each time horizon
            for i, interval in enumerate(intervals):
                logits_start = i * logits_per_interval
                logits_end = logits_start + logits_per_interval

                interval_logits = logits[:, logits_start:logits_end]
                interval_target = target[:, logits_start:logits_end]

                # Loss for this interval
                loss = criterion(interval_logits, interval_target)

                # Accuracy for this interval
                probs = torch.sigmoid(interval_logits)
                preds = (probs > 0.5).float()
                # Compare all 8 bits for all items in batch
                correct = (preds == interval_target).float().mean()

                stats[interval]["total_loss"] += loss.item()
                stats[interval]["correct_bits"] += correct.item()
                stats[interval]["count"] += 1

    # 3. Finalize averages
    results = {}
    for interval, data in stats.items():
        avg_loss = data["total_loss"] / data["count"]
        avg_acc = data["correct_bits"] / data["count"]
        results[interval] = {"loss": avg_loss, "accuracy": avg_acc}

        print(
            f"Interval {interval.name}: Loss {avg_loss:.4f}, Bit-Accuracy {avg_acc:.2%}"
        )


def train_and_export_models(graphs: List[HeteroData], labels: List[List[any]]):
    for graph, label in zip(graphs, labels):
        graph.y = torch.tensor(label, dtype=torch.float)

    random.shuffle(graphs)
    split_idx = int(len(graphs) * TRAIN_DATA_CROSS_RATIO)
    training_data, test_data = graphs[:split_idx], graphs[split_idx:]

    training_loader = DataLoader(training_data, batch_size=32, shuffle=True)
    test_loader = DataLoader(test_data, batch_size=32, shuffle=True)

    for num_layers in LAYER_OPTIONS:
        for convolution in CONVOLUTION_OPTIONS:
            for num_hidden_dims in HIDDEN_DIMENSION_OPTIONS:
                conv_name, args = convolution

                model_name = MODEL_NAME_TEMPLATE.format(
                    num_layers=num_layers,
                    conv_type=conv_name.__name__,
                    num_hidden_dims=num_hidden_dims,
                )

                print(f"Training {model_name}")

                metadata = training_data[0].metadata()
                num_labels = training_data[0].y.shape[-1]

                config = HeteroGNNConfig(
                    edge_specs=convolution,
                    num_layers=num_layers,
                    hidden_channels=num_hidden_dims,
                    out_channels=num_labels,
                    return_node=NodeType.PASS_DESTINATION.value,
                )

                model = GenericHeteroGNN(config=config, metadata=metadata)

                train_single_model(loader=training_loader, model=model)

                print(f"Trained {model_name}")

                evaluate_model(loader=test_loader, model=model)

                model.eval()

                x_dict = training_data[0].x_dict
                edge_dict = training_data[0].edge_index_dict

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
        sys.exit(1)

    labelled_passes = load_and_label_data(sys.argv[1], sys.argv[2], Team.BLUE)
    graphs, labels = process_all_passes(labelled_passes=labelled_passes)

    print("Dataset generated!")

    train_and_export_models(graphs, labels)
