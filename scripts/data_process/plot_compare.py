import plotly.graph_objs as go
import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
from torch.utils.data import Dataset, DataLoader
import numpy as np
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split
from scripts.utils import *
# Load CSV file
def linear_mapping(data, old_min, old_max, new_min, new_max):
    # 计算系数 a 和 b
    a = (new_max - new_min) / (old_max - old_min)
    b = new_min - a * old_min

    # 应用线性映射
    new_data = a * data + b
    return new_data
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


data_path = 'paoqiubiao.csv'
data = pd.read_csv(data_path, encoding='utf-8')
# Display the first few rows of the dataframe

# Assume the last column is the label

features = data.iloc[:, 3:].values

labels = data.iloc[:, 1:3].values
min_angle, max_angle, min_vel, max_vel = 25, 35, 29, 31
angle_labels = linear_mapping(labels[:, 0], min_angle, max_angle, 0, 1.0)
vel_labels = linear_mapping(labels[:, 1], min_vel, max_vel, 0, 1.0)
mapped_labels = np.column_stack((angle_labels, vel_labels))

dataset=SimpleDataset(data.iloc[:, 3:].values, mapped_labels)

data = pd.read_csv(data_path, encoding='utf-8')
speed_model=SimpleMLP(input_dim=2, hidden_dim=100, output_dim=2)
speed_model.load_state_dict(torch.load("save_model_throw_speed.pth"))
distance_model=SimpleMLP(input_dim=2, hidden_dim=100, output_dim=2)
distance_model.load_state_dict(torch.load("save_model_throw.pth"))

speed_output_list=[]
distance_output_list=[]
label_ground_truth=[]
distance_input_list=[]
eval_dataloader = DataLoader(dataset, batch_size=1, shuffle=True)
for feature, mapped_labels in eval_dataloader:
    angle_labels = linear_mapping(mapped_labels.float()[:, 0], 0, 1.0, min_angle, max_angle)
    label_ground_truth.append(angle_labels.item())
    distance_input_list.append(feature[0][1].item())

    output_speed = speed_model(feature[:,2:])
    angle_labels_speed = linear_mapping(output_speed.squeeze().detach().numpy()[0], 0, 1.0, min_angle, max_angle)
    speed_output_list.append(angle_labels_speed.item())

    output_distance=distance_model(feature[:,:2])
    angle_labels_distance = linear_mapping(output_distance.squeeze().detach().numpy()[0], 0, 1.0, 13, 16)
    h, d = feature[0][0].item(), feature[0][1].item()
    angle_analytic, _ = cal_angle_speed(h, d)

    distance_output_list.append(angle_analytic+angle_labels_distance.item())

print(label_ground_truth)
fig = go.Figure()

# 添加折线图
fig.add_trace(go.Scatter(x=distance_input_list, y=label_ground_truth, mode='markers', name='theta ground truth'))
fig.add_trace(go.Scatter(x=distance_input_list, y=distance_output_list, mode='markers', name='theta distance'))
fig.add_trace(go.Scatter(x=distance_input_list, y=speed_output_list, mode='markers', name='theta speed'))
# 添加标题和标签
fig.update_layout(title='',
                  xaxis_title='distance',
                  yaxis_title='theta')
fig.show()

