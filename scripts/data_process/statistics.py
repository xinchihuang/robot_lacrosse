import math

from scripts.chassis_control.parabola_predictor import *
from Plots import *
import os



saved_ball_data=[]
ball_dirs= os.listdir(
    "../saved_data/saved_ball_data_set/")
ball_files = [os.path.join("../saved_data/saved_ball_data_set/", f)  for f in ball_dirs if os.path.isfile(
    os.path.join("../saved_data/saved_ball_data_set/", f))]
saved_robot_data=[]
robt_dirs= os.listdir(
    "../saved_data/saved_robot_data_30_30/")
robot_files = [os.path.join("../saved_data/saved_robot_data_30_30/", f) for f in ball_dirs if os.path.isfile(
    os.path.join("../saved_data/saved_robot_data_30_30/", f))]

for file in ball_files:
    data=np.load(file)
    data = filter_points(data)
    saved_ball_data.append(data)
for file in robot_files:
    data = np.load(file)
    saved_robot_data.append(data)
# print(saved_robot_data)
# print(len(saved_ball_data))
x_data=[]
y_data=[]
x_land=[]
y_land=[]
parabola_filter=ParabolaFitterDirect3D(200)
for i in range(0,len(saved_ball_data)):
    print(i)
    if i==7 or i==8 or i==12 or i==19:
        continue
    # print(saved_ball_data[i])

    ball_data=saved_ball_data[i]
    parabola_filter.fit(ball_data)
    s1,s2=parabola_filter.solve(0.3)
    x1=float(s1[0])
    y1=float(s1[1])
    x2=float(s2[0])
    y2=float(s2[1])
    if y1>0:
        x1,x2=x2,x1
        y1,y2=y2,y1
    new_x=x2-x1
    new_y=y2-y1
    x_land.append(new_x)
    y_land.append(new_y)
    # print(i,s1,s2)
    # x_data.append(float(s1[0]))
    # y_data.append(float(s1[1]))
    # x_data.append(float(s2[0]))
    # y_data.append(float(s2[1]))
points=[x_land,y_land]
points=np.array(points)
print(points)
plot_2D(points)


# prarbola_fitter=ParabolaFitterLSTM(model_path="../save_model.pth",number_of_points=30)
# prarbola_fitter.fit(data)
# plot_3D(data,prarbola_fitter)
# print(prarbola_fitter.solve(0.3))
