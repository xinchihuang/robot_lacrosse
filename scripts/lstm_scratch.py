import os.path

import torch
import torch.nn as nn
import numpy as np

from torch.nn.utils.rnn import pack_padded_sequence, pad_packed_sequence
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim

def load_data():
    dataset=[]
    labelset=[]
    for i in range(100):
        path = "/home/xinchi/catkin_ws/src/robot_lacrosse/scripts/saved_data/" + str(i) + ".npy"
        if os.path.isfile(path):
            data=np.load(path)
            dataset.append(data[:30,:])
            labelset.append(data[-1,:2])
    print(labelset)
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
data,labels=load_data()
# 创建 DataLoader
dataset = SequenceDataset(data, labels)
dataloader = DataLoader(dataset, batch_size=50, shuffle=True, collate_fn=collate_fn)
# 定义 LSTM 模型
class DynamicLSTM(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim, num_layers):
        super(DynamicLSTM, self).__init__()
        self.hidden_dim = hidden_dim
        self.lstm = nn.LSTM(input_dim, hidden_dim, num_layers, batch_first=True)
        self.linear = nn.Linear(hidden_dim, output_dim)

    def forward(self, x, lengths):
        # x: [batch_size, seq_length, input_dim]
        # lengths: [batch_size] 各序列的实际长度

        # 使用 pack_padded_sequence 处理动态序列长度
        packed_input = pack_padded_sequence(x, lengths, batch_first=True, enforce_sorted=False)
        packed_output, (hidden, cell) = self.lstm(packed_input)

        # 使用 pad_packed_sequence 获取输出
        output, _ = pad_packed_sequence(packed_output, batch_first=True)

        # 只获取每个序列最后的输出状态
        idx = (lengths - 1).view(-1, 1).expand(len(lengths), output.size(2)).unsqueeze(1)
        decoded = output.gather(1, idx).squeeze(1)

        # 经过线性层处理得到最终输出
        output = self.linear(decoded)
        return output

model = DynamicLSTM(input_dim=3, hidden_dim=100, output_dim=2, num_layers=3)
optimizer = optim.Adam(model.parameters(), lr=0.001)

# 训练模型
model.train()
for epoch in range(20):  # 训练 5 个 epoch
    for sequences, labels, lengths in dataloader:
        optimizer.zero_grad()
        output = model(sequences, lengths)
        loss = torch.nn.functional.mse_loss(output.squeeze(), labels.float())
        loss.backward()
        optimizer.step()
        print(f"Epoch {epoch}, Loss: {loss.item()}")
        # print(output.squeeze(), labels.float())