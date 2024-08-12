import numpy as np
from numpy.polynomial.polynomial import Polynomial
import sympy as sp
import os
from plotly.offline import plot
import plotly.graph_objects as go
import math
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
from scripts.utils import *

# def plot_parabola(data,check_points=30):
#     x=data[:,0]
#     z=data[:,1]
#     x_p=x[:check_points]
#     z_p=z[:check_points]
#     ransac = make_pipeline(PolynomialFeatures(degree=2), RANSACRegressor())
#     ransac.fit(x_p.reshape(-1, 1), z_p)
#     inlier_mask = ransac.named_steps['ransacregressor'].inlier_mask_
#     outlier_mask = np.logical_not(inlier_mask)
#     line_x = np.linspace(min(x), max(x), 1000).reshape(-1, 1)
#     line_z = ransac.predict(line_x)
#
#     fig = go.Figure()
#
#     # 添加内点
#     fig.add_trace(go.Scatter(x=x.flatten(), y=z,
#                              mode='markers', name='Points', marker=dict(color='blue')))
#
#     # 添加外点
#     # fig.add_trace(go.Scatter(x=x[outlier_mask].flatten(), y=z[outlier_mask],
#     #                          mode='markers', name='Outliers', marker=dict(color='red')))
#
#     # 添加拟合的抛物线
#     fig.add_trace(go.Scatter(x=line_x.flatten(), y=line_z, mode='lines',
#                              name='Fitted Parabola', line=dict(color='green')))
#
#     # 更新图形布局
#     fig.update_layout(title='RANSAC Parabola Fitting and Noise Detection',
#                       xaxis_title='x',
#                       yaxis_title='z',
#                       legend_title='Legend',
#                       width=800,
#                       height=600)
#
#     # 显示图形
#     fig.show()
#






number=30

ball_memory = np.load('../saved_ball_data/' + str(number) + '.npy')
check_point=30
print(len(ball_memory))
ball_memory=ball_memory[:,:]
ball_memory_to_fit=ball_memory[:check_point,:]

x = ball_memory[:, 0]
y = ball_memory[:, 1]
z = ball_memory[:, 2]
m, intercept,inlier_mask  = fit_line(ball_memory)
new_points_to_fit = world_to_parabola_coordinate(ball_memory_to_fit, m, intercept)
new_points_to_fit=point_filters(new_points_to_fit)
new_points = world_to_parabola_coordinate(ball_memory, m, intercept)
new_points=point_filters(new_points)
# plot_parabola(ball_memory)
# plot_parabola(new_points)
a, b, c = fit_parabola(new_points_to_fit)

plot_parabola(new_points,a,b,c)
