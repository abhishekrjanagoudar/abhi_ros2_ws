#!/usr/bin/env python3

import os
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error, mean_absolute_error
import matplotlib.pyplot as plt

# Phase 4: Evaluation
# Computes error metrics and generates actual vs. predicted plots on the validation set.

class RobotDriverNet(nn.Module):
    """ Same model architecture as used in Phase 2 Training """
    def __init__(self, input_size=360, hidden_sizes=[256, 128, 64], output_size=2):
        super(RobotDriverNet, self).__init__()
        self.network = nn.Sequential(
            nn.Linear(input_size, hidden_sizes[0]),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_sizes[0], hidden_sizes[1]),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_sizes[1], hidden_sizes[2]),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_sizes[2], output_size)
        )

    def forward(self, x):
        return self.network(x)

def main():
    csv_path = "/home/abhi_ros2_ws/driving_data.csv"
    model_path = "/home/abhi_ros2_ws/robot_driver_model.pth"
    plot_path = "/home/abhi_ros2_ws/evaluation_results.png"
    
    if not os.path.exists(csv_path) or not os.path.exists(model_path):
        print("Data or model not found. Ensure Phase 1 & 2 are complete.")
        return
        
    print(f"Loading data from {csv_path}...")
    df = pd.read_csv(csv_path)
    
    lidar_cols = [f"lidar_{i}" for i in range(360)]
    X = df[lidar_cols].values
    y = df[["linear_velocity", "angular_velocity"]].values
    
    # Reproduce validation split
    _, X_val, _, y_val = train_test_split(X, y, test_size=0.2, random_state=42)
    
    X_val_tensor = torch.tensor(X_val, dtype=torch.float32)
    y_val_tensor = torch.tensor(y_val, dtype=torch.float32)
    
    print(f"Loading model from {model_path}...")
    model = RobotDriverNet()
    model.load_state_dict(torch.load(model_path, map_location=torch.device('cpu')))
    model.eval()
    
    print("Predicting velocities on the validation set...")
    with torch.no_grad():
        predictions = model(X_val_tensor).numpy()
        
    actual_linear = y_val[:, 0]
    actual_angular = y_val[:, 1]
    
    pred_linear = predictions[:, 0]
    pred_angular = predictions[:, 1]
    
    # Calculate metrics
    mse_linear = mean_squared_error(actual_linear, pred_linear)
    mae_linear = mean_absolute_error(actual_linear, pred_linear)
    
    mse_angular = mean_squared_error(actual_angular, pred_angular)
    mae_angular = mean_absolute_error(actual_angular, pred_angular)
    
    print("\n" + "=" * 50)
    print(" EVALUATION METRICS (Validation Set)")
    print("=" * 50)
    print(f" Linear Velocity  - MSE: {mse_linear:.6f}, MAE: {mae_linear:.6f}")
    print(f" Angular Velocity - MSE: {mse_angular:.6f}, MAE: {mae_angular:.6f}")
    print("=" * 50 + "\n")
    
    # Plotting
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Linear Velocity Scatter
    ax1.scatter(actual_linear, pred_linear, alpha=0.5, color='blue', s=10)
    min_val, max_val = min(actual_linear.min(), pred_linear.min()), max(actual_linear.max(), pred_linear.max())
    ax1.plot([min_val, max_val], [min_val, max_val], 'r--', lw=2, label='Perfect Prediction')
    ax1.set_xlabel("Actual Linear Velocity")
    ax1.set_ylabel("Predicted Linear Velocity")
    ax1.set_title("Linear Velocity: Predicted vs Actual")
    ax1.legend()
    ax1.grid(True)
    
    # Angular Velocity Scatter
    ax2.scatter(actual_angular, pred_angular, alpha=0.5, color='green', s=10)
    min_val, max_val = min(actual_angular.min(), pred_angular.min()), max(actual_angular.max(), pred_angular.max())
    ax2.plot([min_val, max_val], [min_val, max_val], 'r--', lw=2, label='Perfect Prediction')
    ax2.set_xlabel("Actual Angular Velocity")
    ax2.set_ylabel("Predicted Angular Velocity")
    ax2.set_title("Angular Velocity: Predicted vs Actual")
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.savefig(plot_path)
    print(f"Evaluation plots saved to {plot_path}")

if __name__ == '__main__':
    main()
