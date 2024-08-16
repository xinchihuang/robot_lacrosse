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
import plotly.express as px
import pandas as pd

files_and_dirs = os.listdir(
    "../saved_data/saved_ball_data_30_30/")
# Filter out directories, keeping only files
files = [f for f in files_and_dirs if os.path.isfile(
    os.path.join("../saved_data/saved_ball_data_30_30/", f))]
# number = len(files)-1
number = 7

ball_memory = np.load('../saved_data/saved_ball_data_30_30/' + str(number) + '.npy')
i = 30
print(len(ball_memory))
print(ball_memory)
ball_memory=ball_memory[1:,:]

x = ball_memory[:, 0]
y = ball_memory[:, 1]
z = ball_memory[:, 2]

x_p = ball_memory[:i, 0]
y_p = ball_memory[:i, 1]
z_p = ball_memory[:i, 2]

ransac = make_pipeline(PolynomialFeatures(degree=1), RANSACRegressor())
ransac.fit(x_p.reshape(-1, 1), y_p)
inlier_mask = ransac.named_steps['ransacregressor'].inlier_mask_
outlier_mask = np.logical_not(inlier_mask)
line_x = np.linspace(min(x), max(x), 1000).reshape(-1, 1)
line_y = ransac.predict(line_x)

fig = go.Figure()

# 添加内点
fig.add_trace(go.Scatter(x=x.flatten(), y=y,
                         mode='markers', name='Points', marker=dict(color='blue')))

# 添加外点
# fig.add_trace(go.Scatter(x=x[outlier_mask].flatten(), y=y[outlier_mask],
#                          mode='markers', name='Outliers', marker=dict(color='red')))

# 添加拟合的抛物线
fig.add_trace(go.Scatter(x=line_x.flatten(), y=line_y, mode='lines',
                         name='Fitted Line', line=dict(color='green')))

# 更新图形布局
fig.update_layout(title='RANSAC Parabola Fitting and Noise Detection',
                  xaxis_title='x',
                  yaxis_title='y',
                  legend_title='Legend',
                  width=800,
                  height=600)

# 显示图形
fig.show()


ransac = make_pipeline(PolynomialFeatures(degree=2), RANSACRegressor())
ransac.fit(x_p.reshape(-1, 1), z_p)
inlier_mask = ransac.named_steps['ransacregressor'].inlier_mask_
outlier_mask = np.logical_not(inlier_mask)
line_x = np.linspace(min(x), max(x), 1000).reshape(-1, 1)
line_z = ransac.predict(line_x)

fig = go.Figure()

# 添加内点
fig.add_trace(go.Scatter(x=x.flatten(), y=z,
                         mode='markers', name='Points', marker=dict(color='blue')))

# 添加外点
# fig.add_trace(go.Scatter(x=x[outlier_mask].flatten(), y=z[outlier_mask],
#                          mode='markers', name='Outliers', marker=dict(color='red')))

# 添加拟合的抛物线
fig.add_trace(go.Scatter(x=line_x.flatten(), y=line_z, mode='lines',
                         name='Fitted Parabola', line=dict(color='green')))

# 更新图形布局
fig.update_layout(title='RANSAC Parabola Fitting and Noise Detection',
                  xaxis_title='x',
                  yaxis_title='z',
                  legend_title='Legend',
                  width=800,
                  height=600)

# 显示图形
fig.show()

ransac = make_pipeline(PolynomialFeatures(degree=2), RANSACRegressor())
ransac.fit(y_p.reshape(-1, 1), z_p)
inlier_mask = ransac.named_steps['ransacregressor'].inlier_mask_
outlier_mask = np.logical_not(inlier_mask)
line_y = np.linspace(min(y), max(y), 1000).reshape(-1, 1)
line_z = ransac.predict(line_y)

fig = go.Figure()

# 添加内点
fig.add_trace(go.Scatter(x=y.flatten(), y=z,
                         mode='markers', name='Points', marker=dict(color='blue')))

# 添加外点
# fig.add_trace(go.Scatter(x=y[outlier_mask].flatten(), y=z[outlier_mask],
#                          mode='markers', name='Outliers', marker=dict(color='red')))

# 添加拟合的抛物线
fig.add_trace(go.Scatter(x=line_y.flatten(), y=line_z, mode='lines',
                         name='Fitted Parabola', line=dict(color='green')))

# 更新图形布局
fig.update_layout(title='RANSAC Parabola Fitting and Noise Detection',
                  xaxis_title='y',
                  yaxis_title='z',
                  legend_title='Legend',
                  width=800,
                  height=600)
fig.show()
# 显示图形
# Create an index for the points
index = np.arange(len(x))

# Create a DataFrame for plotting
data = {'x': x, 'y': y, 'z': z, 'index': index}
df = pd.DataFrame(data)

# Create a 3D scatter plot with color based on index
fig = px.scatter_3d(df, x='x', y='y', z='z', color='index', color_continuous_scale=px.colors.sequential.Bluered)

# Update the layout to maintain the 1:1:1 aspect ratio
fig.update_layout(scene=dict(
    aspectmode='manual',
    aspectratio=dict(x=1, y=1, z=1)
))

fig.show()