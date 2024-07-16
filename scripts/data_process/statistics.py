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






for number in range(100):
    if os.path.isfile('saved_ball_data/' + str(number) + '.npy'):
        ball_memory = np.load('saved_ball_data/' + str(number) + '.npy')
        check_point=30
        ball_memory=ball_memory[0:,:]
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
        if landing_x_parabola<new_points[-1][0]:
            print(number,"actual:",new_points[-1][0],"predicted:",landing_x_parabola,x1,x2)


