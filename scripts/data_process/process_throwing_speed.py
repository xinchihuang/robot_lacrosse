import math
import os
import numpy as np
from scripts.chassis_control.parabola_predictor import ParabolaFitterRansac


def filter_points(points, threshold=0.1):
    filtered_points = []  # Initialize as a list to collect points
    for i in range(1, len(points) - 1):
        dist_prev = np.linalg.norm(points[i] - points[i - 1])
        dist_next = np.linalg.norm(points[i] - points[i + 1])
        if dist_prev <= threshold or dist_next <= threshold:
            filtered_points.append(points[i])
    return np.array(filtered_points)  # Convert list to NumPy array outside the loop
with open("../paoqiubiao.csv", "w") as csvfile:

    parabola_fitter = ParabolaFitterRansac(100)
    for i in range(0, 100):
        if os.path.isfile(f'../saved_data/saved_ball_data_9_8/{i}.npy'):

            arm_data=np.load(f'../saved_data/saved_arm_data_9_8/{i}.npy')
            desired_angle=arm_data[1]
            desired_speed=arm_data[2]
            data = np.load(f'../saved_data/saved_ball_data_9_8/{i}.npy')
            data = filter_points(data)

            parabola_fitter.fit(data)
            params=parabola_fitter.params
            m, intercept, a, b, c=params

            x1,x2=parabola_fitter.solve(0.3)
            h = -b ** 2 / 4 / a + c-0.3
            d=math.sqrt((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2)
            vx = math.sqrt(9.8 * d ** 2 / 8 * h)
            vz = math.sqrt(2 * 9.8 * h)

            print(f"{i}\t{desired_angle}\t{desired_speed}\t{h}\t{d}\t{vx}\t{vz}")
            csvfile.write(f"{i},{desired_angle},{desired_speed},{h},{d},{vx},{vz}\n")