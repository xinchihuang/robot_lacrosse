import numpy as np
from numpy.polynomial.polynomial import Polynomial
import sympy as sp
import os
from plotly.offline import plot
import plotly.graph_objects as go
import math

files_and_dirs = os.listdir(
    "../saved_ball_data/")
# Filter out directories, keeping only files
files = [f for f in files_and_dirs if os.path.isfile(
    os.path.join("../saved_ball_data/", f))]
number = len(files)
# number=84
ball_memory = np.load('../saved_ball_data/' + str(number-1) + '.npy')
i = 30
print(len(ball_memory))
print(ball_memory)

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
    title="Trajectory of the Ball "+str(number-1),
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
    title="Trajectory of the Ball"+str(number-1),
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

# Create a scatter plot for the data points
trace_data = go.Scatter(x=y, y=z, mode='markers', name='Data Points', marker=dict(color='red'))

# Create a line plot for the fitted parabola
trace_fit = go.Scatter(x=y_fit, y=z_fit, mode='lines', name='Fitted Parabola', marker=dict(color='blue'))

# Define the layout of the plot
layout = go.Layout(
    title="Trajectory of the Ball"+str(number-1),
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