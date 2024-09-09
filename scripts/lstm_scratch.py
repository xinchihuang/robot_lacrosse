import os.path

import torch
import torch.nn as nn
import numpy as np

from torch.nn.utils.rnn import pack_padded_sequence, pad_packed_sequence
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim
from scripts.utils import *
from scripts.chassis_control.chassis_controller import *
def load_data():
    dataset=[]
    labelset=[]
    for i in range(100):
        path = "./saved_ball_data/" + str(i) + ".npy"
        if os.path.isfile(path):
            data=np.load(path)
            ball_memory=data
            selected_ball_memory=data[:30,:]
            m, intercept, inlier_mask = fit_line(selected_ball_memory)
            new_points_to_fit = world_to_parabola_coordinate(selected_ball_memory, m, intercept)
            new_points_to_fit = point_filters(new_points_to_fit)
            new_points = world_to_parabola_coordinate(ball_memory, m, intercept)
            new_points = point_filters(new_points)
            # plot_parabola(ball_memory)
            # plot_parabola(new_points)
            a, b, c = fit_parabola(new_points_to_fit)
            x1, x2 = root(a, b, c - 0.3)

            if x1 == None or x2 == None:
               continue

            x0 = new_points[0][0]
            d1 = (x1 - x0) ** 2
            d2 = (x2 - x0) ** 2
            if max(d1, d2) == d1:
                landing_x_parabola = x1
            else:
                landing_x_parabola = x2
            x, y = landing_x_parabola / math.sqrt(1 + m ** 2), landing_x_parabola * m / math.sqrt(
                1 + m ** 2)

            dataset.append(new_points_to_fit)
            residual=[landing_x_parabola-new_points[-1][0]]
            print(residual)
            labelset.append(residual)
    # print(labelset)
    return dataset,labelset

class SequenceDataset(Dataset):
    def __init__(self, data, labels):
        # 假设 data 是一个列表，每个元素是一个序列（长度可变）
        self.data = data
        self.labels = labels

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        sequence = self.data[idx]
        label = self.labels[idx]
        return torch.tensor(sequence, dtype=torch.float32), label, len(sequence)

def collate_fn(batch):
    batch.sort(key=lambda x: x[2], reverse=True)  # 根据序列长度降序排列
    sequences, labels, lengths = zip(*batch)
    padded_sequences = torch.nn.utils.rnn.pad_sequence(sequences, batch_first=True)
    return padded_sequences, torch.tensor(labels), torch.tensor(lengths)

# 定义 LSTM 模型
class DynamicLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim, num_layers):
        super(DynamicLSTM, self).__init__()
        self.hidden_dim = hidden_dim
        # self.gru=nn.GRU(input_dim, hidden_dim, num_layers)
        self.lstm = nn.GRU(input_dim, hidden_dim, num_layers, batch_first=True)
        self.linear = nn.Linear(hidden_dim, output_dim)

    def forward(self, x, lengths):
        # x: [batch_size, seq_length, input_dim]
        # lengths: [batch_size] 各序列的实际长度

        # 使用 pack_padded_sequence 处理动态序列长度
        packed_input = pack_padded_sequence(x, lengths, batch_first=True, enforce_sorted=False)
        packed_output, (hidden) = self.lstm(packed_input)

        # 使用 pad_packed_sequence 获取输出
        output, _ = pad_packed_sequence(packed_output, batch_first=True)

        # 只获取每个序列最后的输出状态
        idx = (lengths - 1).view(-1, 1).expand(len(lengths), output.size(2)).unsqueeze(1)
        decoded = output.gather(1, idx).squeeze(1)

        # 经过线性层处理得到最终输出
        output = self.linear(decoded)
        return output
if __name__=="__main__":
    data, labels = load_data()
    # 创建 DataLoader
    dataset = SequenceDataset(data, labels)
    dataloader = DataLoader(dataset, batch_size=10, shuffle=True, collate_fn=collate_fn)
    model = DynamicLSTM(input_dim=2, hidden_dim=20, output_dim=1, num_layers=2)
    optimizer = optim.Adam(model.parameters(), lr=0.001)

    # 训练模型
    model.train()
    for epoch in range(5):  # 训练 5 个 epoch
        for sequences, labels, lengths in dataloader:
            print(lengths)
            print(sequences)
            optimizer.zero_grad()
            output = model(sequences, lengths)
            loss = torch.nn.functional.mse_loss(output.squeeze(), labels.float())
            # print(output.squeeze(), labels.float())
            loss.backward()
            optimizer.step()
            print(f"Epoch {epoch}, Loss: {loss.item()}")
    path = "save_model.pth"

    # Save only the state dict
    torch.save(model.state_dict(), path)
            # print(output.squeeze(), labels.float())