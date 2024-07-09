import numpy as np
import math
def check_parabola_point(ball_memory,check_window=20,tolorent_points=3):

    ball_memory = np.array(ball_memory)
    check_point=ball_memory[-1]
    if len(ball_memory) < check_window:
        return False
    outliers_count=0
    previous_slope=float("inf")
    for point in ball_memory[-check_window:-1,:]:
        # print(point)
        slope=(check_point[2]-point[2])/(math.sqrt((check_point[0]-point[0])**2+(check_point[1]-point[1])**2))
        # print(slope)
        if slope < previous_slope:
            previous_slope=slope
        else:
            outliers_count+=1
        if outliers_count>tolorent_points:
            return False
    return True
# ball_memory=np.load("../saved_data/99.npy")
#
# for i in range(1,50):
#     print(check_parabola_point(ball_memory[:i,:]))


