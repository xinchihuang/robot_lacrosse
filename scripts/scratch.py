import numpy as np
from utils import *
print(np.load("./saved_data/saved_arm_data/20.npy"))
h,d=1.538684277,0.5
print(cal_angle_speed(h,d))
# import time
# s=time.time()
# time.sleep(1)
# e=time.time()
# print(e-s)