import numpy as np
from numpy.polynomial.polynomial import Polynomial
import os
from plotly.offline import plot
import plotly.graph_objects as go
import math
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from scripts.utils import *
import os.path

import torch


from torch.nn.utils.rnn import pack_padded_sequence, pad_packed_sequence
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim
from scripts.utils import *
from scripts.chassis_control.chassis_controller import *
from scripts.lstm_scratch import DynamicLSTM

def check_distance(x,y,x_p,y_p):
    return math.sqrt((x-x_p)**2 + (y-y_p)**2)



# Load the state dict (need the model class definition)
model = DynamicLSTM(input_dim=2, hidden_dim=20, output_dim=1, num_layers=2) # Initialize the same model structure
model.load_state_dict(torch.load("save_model.pth"))
count=0

analytical_error=[]
model_error=[]
for number in range(100):
    if os.path.isfile('saved_ball_data_old/' + str(number) + '.npy'):
        ball_memory = np.load('saved_ball_data_old/' + str(number) + '.npy')
        check_point=30
        ball_memory=ball_memory[0:,:]
        ball_memory_to_fit=ball_memory[:check_point,:]

        x = ball_memory[:, 0]
        y = ball_memory[:, 1]
        z = ball_memory[:, 2]
        m, intercept,inlier_mask  = fit_line(ball_memory)
        new_points_to_fit = world_to_parabola_coordinate(ball_memory_to_fit, m, intercept)
        new_points_to_fit = point_filters(new_points_to_fit)

        sequence_length = torch.tensor([len(new_points_to_fit)])
        # print(torch.tensor([new_points_to_fit]))
        residual=model(torch.tensor(new_points_to_fit).float().unsqueeze(0), sequence_length)
        new_points = world_to_parabola_coordinate(ball_memory, m, intercept)
        new_points=point_filters(new_points)
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
        # if landing_x_parabola<new_points[-1][0]:
        #     print(number,"actual:",new_points[-1][0],"predicted:",landing_x_parabola,x1,x2)
        landing_x_parabola_p=landing_x_parabola
        x_p, y_p = landing_x_parabola_p / math.sqrt(1 + m ** 2), landing_x_parabola_p * m / math.sqrt(
            1 + m ** 2)

        landing_x_parabola_m = landing_x_parabola-residual.item()
        x_m, y_m = landing_x_parabola_m / math.sqrt(1 + m ** 2), landing_x_parabola_m * m / math.sqrt(
            1 + m ** 2)
        analytical_error.append(check_distance(ball_memory[-1][0],ball_memory[-1][1],x_p,y_p+intercept))
        model_error.append(check_distance(ball_memory[-1][0],ball_memory[-1][1],x_m,y_m+intercept))
        count+=1


        print(number, "actual:", ball_memory[-1][0],ball_memory[-1][1],"predicted analytical:",x_p,y_p+intercept, "predicted model:",x_m,y_m+intercept)
print("analytical:",np.mean(analytical_error),np.var(analytical_error),"model:",np.mean(model_error),np.var(model_error))