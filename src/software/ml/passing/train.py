import pandas as pd
import numpy as np
import torch
import os
import torch.nn.functional as F
import torch_geometric
from torch_geometric.data import HeteroData
from torch_geometric.nn import SAGEConv, GATConv, to_hetero
from torch_geometric.loader import DataLoader
from uuid import UUID
import onnxruntime
import torch.onnx
from typing import List
from software.ml.models.hetero_gnn.hetero_gnn import GenericHeteroGNN
from software.ml.models.hetero_gnn.config import HeteroGNNConfig
from software.ml.passing.data.types import NodeType
from software.ml.passing.data.build_graph import process_all_passes
from software.ml.passing.data.labelled_passes import LabelledPass

dir_path = os.path.dirname(os.path.realpath(__file__))
onnx_path = os.path.join(dir_path, "onnx")

# --- 1. Data Hydration (CSV to Objects) ---
# Assuming your loading functions from previous steps exist
def load_and_label_data(pass_csv, event_csv, friendly_team) -> List[LabelledPass]:
    # 1. Load raw rows
    # 2. Convert to EventLog and PassLog objects (as discussed)
    # 3. Sort by timestamp
    # 4. Run 'generate_time_interval_labels'
    # For this script, we assume 'labelled_passes' is a List[LabelledPass]
    pass
  
def train_single_model(dataset: HeteroData, model: GenericHeteroGNN, epochs=50, learning_rate=0.01) -> GenericHeteroGNN:
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
      target = data.y.view(logits.size()) # Ensure shapes match exactly
      
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
          num_hidden_dims=num_hidden_dims
        )
        
        metadata = dataset.metadata()
        num_labels = dataset[0].y.shape[-1]
        
        config = HeteroGNNConfig(
          edge_specs=convolution,
          num_layers=num_layers,
          hidden_channels=num_hidden_dims,
          out_channels=num_labels,
          return_node=NodeType.PASS_DESTINATION.value
        )
        
        model = GenericHeteroGNN(
          config=config,
          metadata=metadata
        )
        
        train_single_model(dataset=dataset, model=model)    
        
        model.eval() 
        
        x_dict = dataset[0].x_dict
        edge_dict = dataset[0].edge_index_dict
        
        file_name = os.path.join(onnx_path, f"{model_name}.onnx")
        
        torch.onnx.export(
          model, 
          (x_dict, edge_dict), 
          file_name,
          input_names=['x_dict', 'edge_dict'],
          output_names=['output'],
          dynamic_axes={'x_dict': {0: 'batch'}, 'output': {0: 'batch'}},
          opset_version=15 # Important for complex GNN ops
        )
        
        print(f"Saved {model_name}.onnx")

if __name__ == "__main__":
    labelled_passes = load_and_label_data("passes.csv", "events.csv", MyTeam)
    dataset = process_all_passes(labelled_passes=labelled_passes)
    train_and_export_models(dataset=dataset)