import torch
import torch.nn as nn
import numpy as np
class LSTMModel(nn.Module):
    def __init__(self, input_size, hidden_size, num_layers, output_size):
        super(LSTMModel, self).__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True)
        self.fc = nn.Linear(hidden_size, output_size)

    def forward(self, x):
        h0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)  # Initial hidden state
        c0 = torch.zeros(self.num_layers, x.size(0), self.hidden_size).to(x.device)  # Initial cell state
        out, _ = self.lstm(x, (h0, c0))  # Forward propagate LSTM
        out = self.fc(out[:, -1, :])  # Decode the hidden state of the last time step
        return out
# Generate random data for demo purposes
def clean_data(data):
    first_up=False
    first_down=False
    for i in range(len(data)):
        z=data[i][2]

def generate_data():
    for i in range(100):
        path = "/home/xinchi/catkin_ws/src/robot_lacrosse/scripts/saved_data/" + str(0) + ".npy"
    return seq

# Transform data suitable for LSTM input
def create_inout_sequences(input_data, tw):
    inout_seq = []
    L = len(input_data)
    for i in range(L-tw):
        train_seq = torch.FloatTensor(input_data[i:i+tw])
        train_label = torch.FloatTensor(input_data[i+tw:i+tw+1])
        inout_seq.append((train_seq, train_label))
    return inout_seq

# Example data preparation
seq_length = 5
data = generate_data()
sequences = create_inout_sequences(data, seq_length)
# Hyperparameters
input_size = 1
hidden_size = 50
num_layers = 1
output_size = 1

# Create the LSTM model
model = LSTMModel(input_size, hidden_size, num_layers, output_size)

# Loss and optimizer
criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=0.01)

# Train the model (simplified for demonstration)
for epoch in range(150):
    for seq, labels in sequences:
        optimizer.zero_grad()
        seq = seq.view(-1, seq_length, 1)  # Reshape data for batch size of 1
        y_pred = model(seq)
        loss = criterion(y_pred, labels)
        loss.backward()
        optimizer.step()
    if epoch % 25 == 0:
        print(f'Epoch {epoch} Loss: {loss.item()}')
