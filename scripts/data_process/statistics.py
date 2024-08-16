import math

import numpy as np

from scripts.chassis_control.parabola_predictor import *
from Plots import *
import os
import pandas as pd

saved_ball_data=[]
saved_robot_data=[]

for i in range(0,40):

    robot_file=f"../saved_data/saved_robot_data/{i}.npy"
    robot_data = np.load(robot_file)
    robot_data = filter_points(robot_data)
    saved_robot_data.append(robot_data)
    ball_file = f"../saved_data/saved_ball_data/{i}.npy"
    ball_data = np.load(ball_file)
    ball_data = filter_points(ball_data)
    saved_ball_data.append(ball_data)
success_file=f'../jieqiubiao.csv'
success_df = pd.read_csv(success_file, header=None)
def plot_ball_robot_local(saved_ball_data,saved_robot_data,success_df,parabola_filter):
    parabola_filter = parabola_filter
    ball_start=[]
    ball_end=[]
    robot_start=[]
    robot_end=[]
    success_case=[]
    ball_success=[]
    ball_fail=[]
    ball_almost=[]
    number_of_data=len(saved_ball_data)
    for i in range(1, 20):

        # print(saved_ball_data[i])
        ball_data = saved_ball_data[i]
        parabola_filter.fit(ball_data)
        s1, s2 = parabola_filter.solve(0.3)
        x1 = float(s1[0])
        y1 = float(s1[1])
        x2 = float(s2[0])
        y2 = float(s2[1])
        if y1 > 0:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
        ball_start.append([x1,y1])
        ball_end.append([x2,y2])
        # robot_data=saved_robot_data[i]
        # robot_s=[saved_robot_data[i][0][1],saved_robot_data[i][0][2]]
        # robot_e=[saved_robot_data[i][-1][1],saved_robot_data[i][-1][2]]
        # robot_start.append(robot_s)
        # robot_end.append(robot_e)
        # print(success_df.iat[i,1])
        success_case.append(success_df.iat[i,1])
        rotation=-saved_robot_data[i][-1][4]
        x_local,y_local=x2 - saved_robot_data[i][-1][1], y2 - saved_robot_data[i][-1][2]
        y_local_new=x_local*math.cos(rotation)+y_local*math.sin(rotation)
        x_local_new=-x_local*math.sin(rotation)+y_local*math.cos(rotation)
        # y_local_new = y_local
        # x_local_new = x_local
        if success_df.iat[i,1]=="s":
            ball_success.append([x_local_new,y_local_new])
        elif success_df.iat[i,1]=="a":
            ball_almost.append([x_local_new,y_local_new])
        elif success_df.iat[i,1]=="f":
            ball_fail.append([x_local_new,y_local_new])
    ball_start=np.array(ball_start)

    # ball_end=np.array(ball_end)
    # robot_start=np.array(robot_start)
    # robot_end=np.array(robot_end)
    ball_success,ball_almost,ball_fail=np.array(ball_success),np.array(ball_almost),np.array(ball_fail)

    labels_s = [str(i) for i in range(1, len(ball_start))]
    fig = go.Figure()
    # fig.add_trace(go.Scatter(x=ball_start[:,0], y=ball_start[:,1], mode='markers', name='throwing position',
    #                          marker=dict(size=10, color='blue', symbol='circle', )))
    fig.add_trace(go.Scatter(x=ball_success[:,0], y=ball_success[:,1], mode='markers', name='Successful Catch',
                             marker=dict(size=10, color='green', symbol='circle', )))
    fig.add_trace(go.Scatter(x=ball_almost[:, 0], y=ball_almost[:, 1], mode='markers', name='Hit the Head',
                             marker=dict(size=10, color='yellow', symbol='square', )))
    fig.add_trace(go.Scatter(x=ball_fail[:, 0], y=ball_fail[:, 1], mode='markers', name='Fail',
                             marker=dict(size=10, color='red', symbol='triangle-up', )))



    fig.update_layout(
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(0,0,0,0)',
        # xaxis_title='X',
        # yaxis_title='Y',
        xaxis=dict(
            # title='X(m)',
            title_font=dict(size=18),
            tickfont=dict(size=14),
            scaleanchor = 'y',
            scaleratio = 1,
            linewidth = 2,
            linecolor = 'black',
            showgrid=False,
        ),
        yaxis=dict(
            # title='Y(m)',
            title_font=dict(size=18),
            tickfont=dict(size=14),
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        width=500,
        height=1000,
        showlegend = True,
        legend=dict(
            font=dict(size=16),
            title_font=dict(size=18)
        ),
    )
    fig.show()

def plot_ball_robot_global(saved_ball_data,saved_robot_data,success_df,parabola_filter):
    parabola_filter = parabola_filter
    ball_start=[]
    ball_end=[]
    robot_start=[]
    robot_end=[]
    success_case=[]
    ball_success=[]
    ball_fail=[]
    ball_almost=[]
    number_of_data=len(saved_ball_data)
    for i in range(1, 20):
        if i == 6 or i == 34:
            continue
        # print(saved_ball_data[i])
        ball_data = saved_ball_data[i]
        parabola_filter.fit(ball_data)
        s1, s2 = parabola_filter.solve(0.3)
        x1 = float(s1[0])
        y1 = float(s1[1])
        x2 = float(s2[0])
        y2 = float(s2[1])
        if y1 > 0:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
        ball_start.append([x1,y1])
        ball_end.append([x2,y2])
        # robot_data=saved_robot_data[i]
        robot_s=[saved_robot_data[i][0][1],saved_robot_data[i][0][2]]
        robot_e=[saved_robot_data[i][-1][1],saved_robot_data[i][-1][2]]
        robot_start.append(robot_s)
        robot_end.append(robot_e)
        # print(success_df.iat[i,1])
        success_case.append(success_df.iat[i,1])
        print(success_df.iat[i,1])
        if success_df.iat[i,1]=="s":
            ball_success.append([x2,y2])
        elif success_df.iat[i,1]=="a":
            ball_almost.append([x2,y2])
        elif success_df.iat[i,1]=="f":
            ball_fail.append([x2,y2])
    ball_start=np.array(ball_start)

    ball_end=np.array(ball_end)
    robot_start=np.array(robot_start)
    robot_end=np.array(robot_end)
    ball_success,ball_almost,ball_fail=np.array(ball_success),np.array(ball_almost),np.array(ball_fail)

    print(ball_success)
    labels_s = [str(i) for i in range(1, len(ball_start))]
    fig = go.Figure()
    # fig.add_shape(
    #     type="circle",
    #     xref="x", yref="y",
    #     x0=-0.4, y0=0, x1=0.4, y1=0.8,
    #     line=dict(color="rgba(0, 0, 0, 0.2)"),
    #     fillcolor="rgba(128, 128, 128, 0.1)",
    #     name="Reachable Range"
    # )
    fig.add_shape(
        type="circle",
        xref="x", yref="y",
        x0=-0.4, y0=0.5, x1=0.4, y1=1.3,
        line=dict(color="rgba(0, 0, 0, 0.2)"),
        fillcolor="rgba(128, 128, 128, 0.1)",
        name="Reachable Range"
    )
    # fig.add_shape(
    #     type="circle",
    #     xref="x", yref="y",
    #     x0=-0.1, y0=0.3, x1=0.1, y1=0.5,
    #     line=dict(color="rgba(0, 0, 0, 0.3)"),
    #     fillcolor="rgba(0, 128, 128, 0.2)",
    #     name="Catcher Robot"
    # )
    fig.add_shape(
        type="circle",
        xref="x", yref="y",
        x0=-0.1, y0=0.8, x1=0.1, y1=1.0,
        line=dict(color="rgba(0, 0, 0, 0.3)"),
        fillcolor="rgba(0, 128, 128, 0.2)",
        name="Catcher Robot"
    )
    fig.add_shape(
        type="circle",
        xref="x", yref="y",
        x0=-0.1, y0=-1.3, x1=0.1, y1=-1.5,
        line=dict(color="rgba(0, 0, 0, 0.3)"),
        fillcolor="rgba(0, 0, 128, 0.2)",
        name="Thrower Robot"
    )
    fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             marker=dict(size=20,color="rgba(128, 128, 128, 0.5)"),  name="Reachable Range"))
    fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             marker=dict(size=20,color="rgba(0, 128, 128, 0.5)"),  name="Catcher Robot"))
    fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             marker=dict(size=20,color="rgba(0, 0, 128, 0.5)"),  name="Thrower Robot"))
    # fig.add_trace(go.Scatter(x=ball_start[:,0], y=ball_start[:,1], mode='markers', name='throwing position',
    #                          marker=dict(size=10, color='blue', symbol='circle', )))
    fig.add_trace(go.Scatter(x=ball_success[:,0], y=ball_success[:,1], mode='markers', name='Successful Catch',
                             marker=dict(size=20, color='green', symbol='circle', )))
    fig.add_trace(go.Scatter(x=ball_almost[:, 0], y=ball_almost[:, 1], mode='markers', name='Hit the Head',
                             marker=dict(size=20, color='yellow', symbol='square', )))
    fig.add_trace(go.Scatter(x=ball_fail[:, 0], y=ball_fail[:, 1], mode='markers', name='Fail',
                             marker=dict(size=20, color='red', symbol='triangle-up', )))



    fig.update_layout(
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(0,0,0,0)',
        xaxis_title='X',
        yaxis_title='Y',
        xaxis=dict(
            title='X(m)',
            title_font=dict(size=20),
            tickfont=dict(size=20),
            scaleanchor = 'y',
            scaleratio = 1,
            linewidth = 2,
            linecolor = 'black',
            showgrid=False,
        ),
        yaxis=dict(
            title='Y(m)',
            title_font=dict(size=20),
            tickfont=dict(size=20),
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        width=500,
        height=1000,
        showlegend = True,
        legend=dict(
            font=dict(size=20),
            title_font=dict(size=20)
        ),
    )
    fig.show()
parabola_filter=ParabolaFitterRansac(200)
plot_ball_robot_global(saved_ball_data,saved_robot_data,success_df,parabola_filter=parabola_filter)
# plot_ball_robot_local(saved_ball_data,saved_robot_data,success_df,parabola_filter=parabola_filter)
def cal_ball_landing(saved_ball_data):

    x_data=[]
    y_data=[]
    x_land=[]
    y_land=[]
    parabola_filter=ParabolaFitterDirect3D(200)
    for i in range(0,len(saved_ball_data)):
        print(i)
        if i==27 or i==30:
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
    return points
def cal_robot_distribution(saved_robot_data):
    x_s=[]
    y_s=[]
    x_e = []
    y_e = []
    for i in range(1,len(saved_robot_data)):
        print(len(saved_robot_data[i]),i)
        # for j in range(len(saved_robot_data[i])-1,len(saved_robot_data[i])):
        x_s.append(saved_robot_data[i][0][1])
        y_s.append(saved_robot_data[i][0][2])
        x_e.append(saved_robot_data[i][-1][1])
        y_e.append(saved_robot_data[i][-1][2])
    return [x_s,y_s],[x_e,y_e]
# ball_points=cal_ball_landing(saved_ball_data)
# plot_2D(ball_points,"ball")


# prarbola_fitter=ParabolaFitterLSTM(model_path="../save_model.pth",number_of_points=30)
# prarbola_fitter.fit(data)
# plot_3D(data,prarbola_fitter)
# print(prarbola_fitter.solve(0.3))
