import random
import sys
import torch
import os
from sklearn.metrics import f1_score
from torch_geometric.data import HeteroData
from torch_geometric.nn import SAGEConv, GATConv
from torch_geometric.loader import DataLoader
import torch.onnx
import numpy as np
from typing import List, Any
from dataclasses import fields
from software.ml.models.hetero_gnn.hetero_gnn import GenericHeteroGNN
from software.ml.models.hetero_gnn.config import HeteroGNNConfig
from software.ml.passing.data.types import NodeType
from software.ml.passing.data.build_graph import process_all_passes
from software.ml.passing.data.pass_result import generate_pass_results
from software.ml.passing.data.labelled_passes import LabelledPass, label_passes, Label
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
        event_logs=event_logs[0:10000], pass_logs=pass_logs, friendly_team=friendly_team
    )

    print("Pass Results Generated!")

    labelled_passes = label_passes(pass_results)

    print("Passes labelled!")

    return labelled_passes

def calculate_label_weights(labelled_passes: list[LabelledPass]):
    """
    Calculates the positive weights for BCEWithLogitsLoss for each label type
    based on the actual frequency of events in the dataset.
    """
    # 1. Flatten all labels into a single large 2D array [num_samples, num_bits]
    all_label_tensors = []
    
    for labelled_pass in labelled_passes:
        # Concatenate labels for all intervals (1s, 5s, 10s, etc.) 
        # into one long vector for this pass
        pass_bits = []
        
        # iterate through intervals in order
        for interval in sorted(labelled_pass.labels.keys(), key=lambda x: x.value):
            label = labelled_pass.labels[interval]
            pass_bits.extend([
                label.has_score_changed,
                label.has_enemy_score_changed,
                label.have_yellow_cards_changed,
                label.have_red_cards_changed,
                label.has_possession_changed,
                label.have_shots_on_net_changed,
                label.is_enemy_possession,
                label.has_ball_in_half_changed,
                label.is_ball_in_enemy_half
            ])
        all_label_tensors.append(pass_bits)

    # Convert to numpy for easy counting
    labels_np = np.array(all_label_tensors, dtype=np.float32)
    
    # 2. Calculate weights for each bit
    # pos_weight = (num_negative_samples) / (num_positive_samples)
    num_positives = np.sum(labels_np, axis=0)
    num_negatives = labels_np.shape[0] - num_positives
    
    # Avoid division by zero if an event never happened in the dataset
    # We use 1.0 as a default weight for those cases
    label_weights = np.divide(num_negatives, num_positives, 
                            out=np.ones_like(num_positives), 
                            where=num_positives != 0)
    
    print(f"Weights are: {label_weights}")

    # 3. Convert to Torch Tensor
    return torch.tensor(label_weights, dtype=torch.float32)

def undersample_passes(all_passes: list[LabelledPass], boring_keep_ratio: float = 0.1):
    """
    Filters the dataset to keep all 'interesting' passes and a 
    fraction of the 'boring' ones.
    """
    interesting_passes = []
    boring_passes = []

    for labelled_pass in all_passes:
        # Check if ANY label in ANY interval is True
        # Note: We exclude 'is_enemy_possession' and 'is_ball_in_enemy_half' 
        # from the 'interesting' check because they are static states, not events.
        is_interesting = False
        for interval_label in labelled_pass.labels.values():
            # We check specific event-based flags
            if any([
                interval_label.has_score_changed,
                interval_label.has_enemy_score_changed,
                interval_label.have_yellow_cards_changed,
                interval_label.have_red_cards_changed,
                interval_label.has_possession_changed,
                interval_label.have_shots_on_net_changed,
                interval_label.has_ball_in_half_changed
            ]):
                is_interesting = True
                break
        
        if is_interesting:
            interesting_passes.append(labelled_pass)
        else:
            boring_passes.append(labelled_pass)

    # Sample a percentage of the boring passes
    num_boring_to_keep = int(len(boring_passes) * boring_keep_ratio)
    sampled_boring = random.sample(boring_passes, num_boring_to_keep)

    combined = interesting_passes + sampled_boring
    random.shuffle(combined) # Shuffle so the model doesn't see all goals at once
    
    print(f"Original: {len(all_passes)} | Interesting: {len(interesting_passes)} | Boring Kept: {len(sampled_boring)}")
    return combined

def train_single_model(
    loader: DataLoader, 
    model: GenericHeteroGNN, 
    label_weights: torch.Tensor,
    epochs=50, 
    learning_rate=0.01
) -> GenericHeteroGNN:
    # 1. Setup the "Judge" (Loss Function) and the "Optimizer" (Weight Updater)
    # BCEWithLogitsLoss combines a Sigmoid layer and the BCELoss in one single class.
    # It's more numerically stable than using a plain Sigmoid followed by BCELoss.
    criterion = torch.nn.BCEWithLogitsLoss(pos_weight=label_weights)
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
    model.eval()

    intervals = [
        interval for interval in PassLogType if interval != PassLogType.RESULT_0S
    ]

    interval_preds = {interval: [] for interval in intervals}
    interval_targets = {interval: [] for interval in intervals}

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

                interval_targets[interval].append(interval_target)

                preds = (torch.sigmoid(interval_logits) > 0.5).float()
                interval_preds[interval].append(preds)
    
    label_names = [label.name for label in fields(Label)]

    for _, interval in enumerate(intervals):
        preds = interval_preds[interval]
        targets = interval_targets[interval]
        
        pred_matrix = torch.cat(preds, dim=0).cpu().numpy()
        target_matrix = torch.cat(targets, dim=0).cpu().numpy()
   
        print(f"\n--- Interval: {interval.name} ---")

        for j, name in enumerate(label_names):
            f1 = f1_score(target_matrix[:, j], pred_matrix[:, j], zero_division=0)
            print(f"{name:20} | F1-Score: {f1:.4f}")

def train_and_export_models(graphs: List[HeteroData], labels: List[List[Any]], label_weights: torch.Tensor):
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

                train_single_model(loader=training_loader, model=model, label_weights=label_weights)

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
    
    label_weights = calculate_label_weights(labelled_passes=labelled_passes)
    
    undersampled_passes = undersample_passes(labelled_passes)
    
    graphs, labels = process_all_passes(labelled_passes=undersampled_passes)

    print("Dataset generated!")

    train_and_export_models(graphs, labels, label_weights)
