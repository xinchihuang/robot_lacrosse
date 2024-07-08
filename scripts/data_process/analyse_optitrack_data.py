import numpy as np
from numpy.polynomial.polynomial import Polynomial
import sympy as sp
import os
from plotly.offline import plot
import plotly.graph_objects as go
import math
# with open("book2.csv") as file:
#     line_count=0
#     ball_memory=[]
#     while True:
#         line=file.readline()
#         if not line:
#             break
#         line_count+=1
#         if line_count<=1:
#             continue
#         data_list=line.split(",")
#         ball_memory.append([float(data_list[4]),float(data_list[5]),float(data_list[6]),float(data_list[0])])
def root(a,b,c):
    return (-b + math.sqrt(b**2 - 4*a*c))/2/a,(-b - math.sqrt(b**2 - 4*a*c))/2/a
for name in range(107,108):
    try:
        name=107
        ball_memory=np.load('saved_data/'+str(name)+'.npy')
        i=30
        print(len(ball_memory))

        x = ball_memory[:, 0]
        y = ball_memory[:, 1]
        z = ball_memory[:, 2]

        # Fit a second-degree polynomial (parabola) to the y and z coordinates
        coefficients = np.polyfit(x[:i], y[:i], 1)
        a, b = coefficients  # Extract coefficients

        # Generate y values for the fit
        x_fit = np.linspace(min(x), max(x), 400)
        y_fit = a * x_fit + b  # Calculate fitted z values using the polynomial

        # Create a scatter plot for the data points
        trace_data = go.Scatter(x=x, y=y, mode='markers', name='Data Points', marker=dict(color='red'))

        # Create a line plot for the fitted parabola
        trace_fit = go.Scatter(x=x_fit, y=y_fit, mode='lines', name='Fitted Parabola', marker=dict(color='blue'))

        # Define the layout of the plot
        layout = go.Layout(
            title="Trajectory of the Ball",
            xaxis=dict(
                title='x',
                constrain='domain'  # Keeps x-axis within the domain of the data
            ),
            yaxis=dict(
                title='y',
                # scaleanchor="y",  # Ensures y-axis is scaled to x-axis
                # scaleratio=1  # Keeps 1:1 aspect ratio
            ),
            showlegend=True,
            autosize=False,
            width=1000,
            height=1000
        )

        # Create the figure with data and layout
        fig = go.Figure(data=[trace_data, trace_fit], layout=layout)

        # Show the plot
        fig.show()

        x = ball_memory[:, 0]
        y = ball_memory[:, 1]
        z = ball_memory[:, 2]

        # Fit a second-degree polynomial (parabola) to the y and z coordinates
        coefficients = np.polyfit(x[:i], z[:i], 2)
        a, b, c = coefficients  # Extract coefficients

        # Generate y values for the fit
        x_fit = np.linspace(min(x), max(x), 400)
        z_fit = a * x_fit ** 2 + b * x_fit + c  # Calculate fitted z values using the polynomial

        # Create a scatter plot for the data points
        trace_data = go.Scatter(x=x, y=z, mode='markers', name='Data Points', marker=dict(color='red'))

        # Create a line plot for the fitted parabola
        trace_fit = go.Scatter(x=x_fit, y=z_fit, mode='lines', name='Fitted Parabola', marker=dict(color='blue'))

        # Define the layout of the plot
        layout = go.Layout(
            title="Trajectory of the Ball",
            xaxis=dict(
                title='x',
                constrain='domain'
            ),
            yaxis=dict(
                title='z',
                # scaleanchor="y",  # Anchors z-axis scale to x-axis
                # scaleratio=1
            ),
            showlegend=True,
            autosize=False,
            width=1000,
            height=1000
        )

        # Create the figure with data and layout
        fig = go.Figure(data=[trace_data, trace_fit], layout=layout)

        # Show the plot
        fig.show()



        x = ball_memory[:, 0]
        y = ball_memory[:, 1]
        z = ball_memory[:, 2]

        # Fit a second-degree polynomial (parabola) to the y and z coordinates
        coefficients = np.polyfit(y[:i], z[:i], 2)
        a, b, c = coefficients  # Extract coefficients

        # Generate y values for the fit
        y_fit = np.linspace(min(y), max(y), 400)
        z_fit = a * y_fit ** 2 + b * y_fit + c  # Calculate fitted z values using the polynomial

        print(root(a,b,c-0.3))
        # Create a scatter plot for the data points
        trace_data = go.Scatter(x=y, y=z, mode='markers', name='Data Points', marker=dict(color='red'))

        # Create a line plot for the fitted parabola
        trace_fit = go.Scatter(x=y_fit, y=z_fit, mode='lines', name='Fitted Parabola', marker=dict(color='blue'))

        # Define the layout of the plot
        layout = go.Layout(
            title="Trajectory of the Ball",
            xaxis=dict(
                title='y',
                constrain='domain'
            ),
            yaxis=dict(
                title='z',
                # scaleanchor="y",  # Anchors z-axis scale to y-axis
                # scaleratio=1
            ),
            showlegend=True,
            autosize=False,
            width=1000,
            height=1000
        )

        # Create the figure with data and layout
        fig = go.Figure(data=[trace_data, trace_fit], layout=layout)

        # Show the plot
        fig.show()

    except:
        pass
# print(ball_memory)
# start_time=ball_memory[0][3]
# A=[]
# B=[]
# ball_memory=np.array(ball_memory)
# for i in range(8,len(ball_memory)):
#     x=ball_memory[:i,0]
#     y=ball_memory[:i,1]
#     z=ball_memory[:i,2]
#     coefficients = np.polyfit(x, y, 1)  # 返回值是高阶到低阶的系数列表
#
#     # 获取系数
#     a, b,  = coefficients
#     # 用拟合的抛物线方程生成 y 值
#     x_fit = np.linspace(min(x), max(x), 400)
#     y_fit = a * x_fit + b
#
#     # 绘制数据点和拟合的抛物线
# plt.scatter(x, y, color='red', label='Data Points')
# plt.plot(x_fit, y_fit, color='blue', label='Fitted Parabola')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.axis('equal')
# plt.legend()
# plt.show()
# for i in range(8,len(ball_memory)):
#     x=ball_memory[:i,0]
#     y=ball_memory[:i,1]
#     z=ball_memory[:i,2]
#     coefficients = np.polyfit(x, z, 2)  # 返回值是高阶到低阶的系数列表
#
#     # 获取系数
#     a, b, c  = coefficients
#     # 用拟合的抛物线方程生成 y 值
#     x_fit = np.linspace(min(x), max(x), 400)
#     z_fit = a * x_fit ** 2 + b * x_fit + c
#
#     # 绘制数据点和拟合的抛物线
# plt.scatter(x, z, color='red', label='Data Points')
# plt.plot(x_fit, z_fit, color='blue', label='Fitted Parabola')
# plt.xlabel('x')
# plt.ylabel('z')
# plt.axis('equal')
# plt.legend()
# plt.show()