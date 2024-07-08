import math

import matplotlib.pyplot as plt
import numpy as np

pose_array=np.load("saved_pose.npy")
# 圆的参数
h = 0  # 圆心x坐标
k = 0  # 圆心y坐标
r = 0.05  # 半径

# 生成角度
theta = np.linspace(0, 2*np.pi, 100)

# 圆的x和y坐标
x = h + r * np.cos(theta)
y = k + r * np.sin(theta)

# 绘制圆
plt.figure(figsize=(6,6))
plt.plot(x, y)
plt.title('Circle with center ({}, {}) and radius {}'.format(h, k, r))
plt.xlabel('X')
plt.ylabel('Y')
plt.axis('equal')  # 确保x轴和y轴的比例相同，以保持圆形
print(len(pose_array))
for i in range(len(pose_array)):
    pose=pose_array[i]
    theta=pose[5]
    # print(pose)

    ball_relative_pos_x=pose[0]-(pose[3])
    ball_relative_pos_y=pose[1]-(pose[4])
    if abs(ball_relative_pos_x)>0.5:
        continue
    plt.scatter(ball_relative_pos_x,ball_relative_pos_y)
plt.grid(True)
plt.show()
