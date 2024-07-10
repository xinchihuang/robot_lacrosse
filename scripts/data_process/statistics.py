import math
import os.path

import numpy as np
from scripts.chassis_control.chassis_controller import landing_point_predictor

for name in range(0,10):
        if not os.path.isfile('../simulate_parabola/'+str(name)+'.npy'):
                continue
        ball_memory=np.load('../simulate_parabola/'+str(name)+'.npy')
        i=3
        ball_memory_for_predict=ball_memory[0:i,:]

        arm_height=ball_memory[-1][2]
        landing_point_predict=landing_point_predictor(ball_memory_for_predict,arm_height)
        # print(landing_point_predict)
        print(name)
        print("actual", ball_memory[-1])
        print("predict",landing_point_predict)
        print(math.sqrt((landing_point_predict[0]-ball_memory[-1][0])**2+(landing_point_predict[1]-ball_memory[-1][1])**2))
