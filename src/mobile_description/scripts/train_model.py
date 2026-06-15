#!/usr/bin/env python3
"""
train_model.py
--------------
Phase 2 of the imitation-learning pipeline.

Loads the recorded driving_data.csv, trains a PyTorch MLP to predict
linear and angular velocities from 360 normalized LiDAR readings,
and saves the trained model along with a loss curve plot.

Usage
-----
python3 train_model.py
"""

import os
import pandas as pd
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CSV_FILENAME = "driving_data.csv"
MODEL_FILENAME = "robot_driver_model.pth"
PLOT_FILENAME = "loss_curve.png"

# Hyperparameters
BATCH_SIZE = 64
EPOCHS = 50
LEARNING_RATE = 0.001
DROPOUT_RATE = 0.2


# ===========================================================================
# Dataset Definition
# ===========================================================================
class DrivingDataset(Dataset):
    """Custom Dataset for loading preprocessed LiDAR and velocity pairs."""

    def __init__(self, X: np.ndarray, y: np.ndarray):
        self.X = torch.tensor(X, dtype=torch.float32)
        self.y = torch.tensor(y, dtype=torch.float32)

    def __len__(self) -> int:
        return len(self.X)

    def __getitem__(self, idx: int):
        return self.X[idx], self.y[idx]


# ===========================================================================
# Model Architecture
# ===========================================================================
class RobotDriverNet(nn.Module):
    """
    Multi-Layer Perceptron (MLP) for predicting command velocities from LiDAR scans.
    
    Inputs:
        360 normalized LiDAR range values [0, 1]
    Outputs:
        2 predicted continuous velocities (linear_velocity, angular_velocity)
    """

    def __init__(self):
        super().__init__()
        # Structure implements: Linear -> ReLU -> Dropout -> Linear -> ReLU -> Dropout -> Linear
        # with hidden sizes 256, 128, 64 and output size 2.
        self.network = nn.Sequential(
            nn.Linear(360, 256),
            nn.ReLU(),
            nn.Dropout(p=DROPOUT_RATE),
            nn.Linear(256, 128),
            nn.ReLU(),
            nn.Dropout(p=DROPOUT_RATE),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Dropout(p=DROPOUT_RATE),
            nn.Linear(64, 2)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.network(x)


# ===========================================================================
# Training Loop
# ===========================================================================
def main():
    # Set PyTorch device
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # ── Load Data ──────────────────────────────────────────────────────────
    csv_path = os.path.join(os.getcwd(), CSV_FILENAME)
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found. Please record data first.")
        return

    print(f"Loading data from {csv_path}...")
    df = pd.read_csv(csv_path)
    print(f"Total dataset size: {len(df)} rows")

    # Extract features (first 360 columns) and targets (last 2 columns)
    lidar_cols = [f"lidar_{i}" for i in range(360)]
    X = df[lidar_cols].values
    y = df[["linear_velocity", "angular_velocity"]].values

    # ── Split Data (80% Train, 20% Val) ────────────────────────────────────
    X_train, X_val, y_train, y_val = train_test_split(
        X, y, test_size=0.2, random_state=42
    )
    print(f"Train samples: {len(X_train)}  |  Validation samples: {len(X_val)}")

    train_dataset = DrivingDataset(X_train, y_train)
    val_dataset = DrivingDataset(X_val, y_val)

    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False)

    # ── Initialize Model, Loss, Optimizer ─────────────────────────────────
    model = RobotDriverNet().to(device)
    criterion = nn.MSELoss()
    optimizer = optim.Adam(model.parameters(), lr=LEARNING_RATE)

    # Logging structures
    train_losses = []
    val_losses = []

    print("\nStarting Training...")
    print("-" * 50)

    for epoch in range(EPOCHS):
        # Training Phase
        model.train()
        running_train_loss = 0.0
        for batch_x, batch_y in train_loader:
            batch_x, batch_y = batch_x.to(device), batch_y.to(device)

            optimizer.zero_grad()
            predictions = model(batch_x)
            loss = criterion(predictions, batch_y)
            loss.backward()
            optimizer.step()

            running_train_loss += loss.item() * batch_x.size(0)

        epoch_train_loss = running_train_loss / len(X_train)
        train_losses.append(epoch_train_loss)

        # Validation Phase
        model.eval()
        running_val_loss = 0.0
        with torch.no_grad():
            for batch_x, batch_y in val_loader:
                batch_x, batch_y = batch_x.to(device), batch_y.to(device)
                predictions = model(batch_x)
                loss = criterion(predictions, batch_y)
                running_val_loss += loss.item() * batch_x.size(0)

        epoch_val_loss = running_val_loss / len(X_val)
        val_losses.append(epoch_val_loss)

        # Print loss statistics
        print(
            f"Epoch {epoch + 1:02d}/{EPOCHS:02d} | "
            f"Train Loss: {epoch_train_loss:.6f} | "
            f"Val Loss: {epoch_val_loss:.6f}"
        )

    print("-" * 50)
    print("Training finished.")

    # ── Save Model ─────────────────────────────────────────────────────────
    model_path = os.path.join(os.getcwd(), MODEL_FILENAME)
    torch.save(model.state_dict(), model_path)
    print(f"Model weights saved to {model_path}")

    # ── Plot Loss Curves ───────────────────────────────────────────────────
    plt.figure(figsize=(10, 6))
    plt.plot(range(1, EPOCHS + 1), train_losses, label="Training Loss")
    plt.plot(range(1, EPOCHS + 1), val_losses, label="Validation Loss")
    plt.xlabel("Epoch")
    plt.ylabel("MSE Loss")
    plt.title("Imitation Learning Model Training Loss")
    plt.legend()
    plt.grid(True)

    plot_path = os.path.join(os.getcwd(), PLOT_FILENAME)
    plt.savefig(plot_path)
    plt.close()
    print(f"Loss curve plot saved to {plot_path}")


if __name__ == "__main__":
    main()
