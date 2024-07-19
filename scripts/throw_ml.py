import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
from torch.utils.data import Dataset, DataLoader
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
# Load CSV file
data_path = 'throw_data.csv'
data = pd.read_csv(data_path)
# Display the first few rows of the dataframe




# Assume the last column is the label

train_features = data.iloc[:70, 3:].values

train_labels = data.iloc[:70, 1:3].values
eval_features = data.iloc[70:, 3:].values
eval_labels = data.iloc[70:, 1:3].values

def linear_mapping(data, old_min, old_max, new_min, new_max):
    # 计算系数 a 和 b
    a = (new_max - new_min) / (old_max - old_min)
    b = new_min - a * old_min

    # 应用线性映射
    new_data = a * data + b
    return new_data

train_mapped_x_labels = linear_mapping(train_labels[:, 0], -12.0, -10.0,0, 1.0)
train_mapped_y_labels = linear_mapping(train_labels[:, 1],55, 65, 0, 1.0)
train_mapped_labels = np.column_stack((train_mapped_x_labels, train_mapped_y_labels))

class SimpleDataset(Dataset):
    def __init__(self, features, labels):
        # 假设 data 是一个列表，每个元素是一个序列（长度可变）
        self.features = features
        self.labels = labels

    def __len__(self):
        return len(self.features)

    def __getitem__(self, idx):
        sequence = self.features[idx]
        label = self.labels[idx]
        return torch.tensor(sequence, dtype=torch.float32), torch.tensor(label)


class SimpleMLP(nn.Module):
    def __init__(self, input_dim=2, hidden_dim=20, output_dim=2):
        super(SimpleMLP, self).__init__()
        self.fc1 = nn.Linear(input_dim, hidden_dim)  # First fully connected layer
        self.relu = nn.ReLU()                         # Activation function
        self.fc2 = nn.Linear(hidden_dim, output_dim)# Second fully connected layer

    def forward(self, x):
        out = self.fc1(x)
        out = self.relu(out)
        out = self.fc2(out)
        return out
dataset = SimpleDataset(train_features, train_mapped_labels)
dataloader = DataLoader(dataset, batch_size=10, shuffle=True)
input_dim=2
hidden_dim=20
output_dim=2
# Create the model
model = SimpleMLP(input_dim, hidden_dim, output_dim)
optimizer = optim.Adam(model.parameters(), lr=0.001)

# 训练模型
model.train()
for epoch in range(50):  # 训练 5 个 epoch
    for sequences, labels in dataloader:
        optimizer.zero_grad()
        output = model(sequences)
        loss = torch.nn.functional.mse_loss(output.squeeze(), labels.float())
        # print(output.squeeze(), labels.float())
        loss.backward()
        optimizer.step()
        print(f"Epoch {epoch}, Loss: {loss.item()}")
path = "save_model_throw.pth"

eval_mapped_x_labels = linear_mapping(eval_labels[:, 0], -12.0, -10.0,0, 1.0)
eval_mapped_y_labels = linear_mapping(eval_labels[:, 1],55, 65, 0, 1.0)
eval_mapped_labels = np.column_stack((eval_mapped_x_labels, eval_mapped_y_labels))
eval_set=SimpleDataset(eval_features, eval_mapped_labels)
eval_dataloader = DataLoader(eval_set, batch_size=1, shuffle=True)
for feature, labels in eval_dataloader:
        optimizer.zero_grad()
        output = model(feature)
        output_x_labels = linear_mapping(output.squeeze().detach().numpy()[0], 0, 1.0,-12.0, -10.0,)
        output_y_labels = linear_mapping(output.squeeze().detach().numpy()[1],  0, 1.0,55, 65)
        output_labels = np.column_stack((output_x_labels, output_y_labels))
        x_labels = linear_mapping(labels.float()[:, 0], 0, 1.0, -12.0, -10.0, )
        y_labels = linear_mapping(labels.float()[:, 1], 0, 1.0, 55, 65)
        labels = np.column_stack((x_labels, y_labels))
        print(f"out put {output_labels}, Labl: {labels }")
model.eval()

# Save only the state dict
torch.save(model.state_dict(), path)
# print(output.squeeze(), labels.float())