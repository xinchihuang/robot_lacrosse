import math

import numpy as np

from scripts.chassis_control.parabola_predictor import *
from Plots import *
import os
import pandas as pd
from scripts.chassis_control.chassis_controller import optitrack_coordinate_to_world_coordinates, central_controller,landing_point_predictor,landing_point_predictor_lstm
from scripts.lstm_scratch import DynamicLSTM

saved_ball_data=[]
saved_robot_data=[]

for i in range(0,100):

    robot_file=f"../saved_robot_data/{i}.npy"
    robot_data = np.load(robot_file)
    robot_data = filter_points(robot_data)
    saved_robot_data.append(robot_data)
    ball_file = f"../saved_ball_data/{i}.npy"
    ball_data = np.load(ball_file)
    ball_data = filter_points(ball_data)
    saved_ball_data.append(ball_data)
success_file=f'../jieqiubiao_9_8.csv'
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
    for i in range(0,100):

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
    fig.add_shape(
        type="circle",
        xref="x", yref="y",
        x0=-0.5, y0=-0.5, x1=0.5, y1=0.5,
        line=dict(color="rgba(0, 0, 0, 0.2)"),
        fillcolor="rgba(128, 128, 128, 0.1)",
        name="Reachable Range"
    )
    # fig.add_shape(
    #     type="circle",
    #     xref="x", yref="y",
    #     x0=-0.02, y0=-0.02, x1=0.02, y1=0.02,
    #     line=dict(color="rgba(0, 0, 0, 1)"),
    #     fillcolor="rgba(0, 0, 0, 1)",
    #     name="Reachable Range"
    # )
    # fig.add_trace(go.Scatter(x=ball_start[:,0], y=ball_start[:,1], mode='markers', name='throwing position',
    #                          marker=dict(size=10, color='blue', symbol='circle', )))
    # fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             # marker=dict(size=20, color="rgba(0, 0, 0, 1)"), name="Catcher Initial Position"))
    fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             marker=dict(size=20, color="rgba(128, 128, 128, 0.5)"), name="Reachable Range"))
    fig.add_trace(go.Scatter(x=ball_fail[:, 0], y=ball_fail[:, 1], mode='markers', name='Fail',
                             marker=dict(size=12, color='red', symbol='triangle-up', )))
    fig.add_trace(go.Scatter(x=ball_almost[:, 0], y=ball_almost[:, 1], mode='markers', name='Successful Contact',
                             marker=dict(size=12, color='orange', symbol='square', )))
    fig.add_trace(go.Scatter(x=ball_success[:, 0], y=ball_success[:, 1], mode='markers', name='Successful Catch',
                             marker=dict(size=12, color='green', symbol='circle', )))
    # fig.update_xaxes(range=[-1, 1])
    # fig.update_yaxes(range=[-1, 1])

    fig.update_layout(
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(0,0,0,0)',
        xaxis_title='Y (m)',
        yaxis_title='X (m)',
        xaxis=dict(
            # title='X(m)',
            title_font=dict(size=28),
            tickfont=dict(size=28),
            scaleanchor = 'y',
            scaleratio = 1,
            linewidth = 2,
            linecolor = 'black',
            showgrid=False,
        ),
        yaxis=dict(
            # title='Y(m)',
            title_font=dict(size=28),
            tickfont=dict(size=28),
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        width=700,
        height=700,
        showlegend = True,
        legend=dict(
            font=dict(size=23),
            title_font=dict(size=23),
            orientation='h',  # 指定图例水平放置
            xanchor='center',  # 中心对齐
            yanchor='bottom',  # 底部对齐
            x=0.5,  # 中心位置
            y=1.0  # 调整位置，负值为向下
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
    for i in range(0,100):
        # if i == 6 or i == 34:
        #     continue
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
            ball_success.append([x2,y2+1])
        elif success_df.iat[i,1]=="a":
            ball_almost.append([x2,y2+1])
        elif success_df.iat[i,1]=="f":
            ball_fail.append([x2,y2+1])
    ball_start=np.array(ball_start)

    ball_end=np.array(ball_end)
    robot_start=np.array(robot_start)
    print(robot_start)
    robot_end=np.array(robot_end)
    ball_success,ball_almost,ball_fail=np.array(ball_success),np.array(ball_almost),np.array(ball_fail)

    print(ball_success)
    labels_s = [str(i) for i in range(1, len(ball_start))]
    fig = go.Figure()
    # fig.add_shape(
    #     type="circle",
    #     xref="x", yref="y",
    #     x0=-0.4, y0=1.05, x1=0.4, y1=1.85,
    #     line=dict(color="rgba(0, 0, 0, 0.2)"),
    #     fillcolor="rgba(128, 128, 128, 0.1)",
    #     name="Reachable Range"
    # )
    fig.add_shape(
        type="circle",
        xref="x", yref="y",
        x0=-0.4, y0=1.5, x1=0.4, y1=2.3,
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
    # fig.add_shape(
    #     type="circle",
    #     xref="x", yref="y",
    #     x0=-0.1, y0=0.8, x1=0.1, y1=1.0,
    #     line=dict(color="rgba(0, 0, 0, 0.3)"),
    #     fillcolor="rgba(0, 128, 128, 0.2)",
    #     name="Catcher Robot"
    # )
    # fig.add_shape(
    #     type="circle",
    #     xref="x", yref="y",
    #     x0=-0.1, y0=-1.3, x1=0.1, y1=-1.5,
    #     line=dict(color="rgba(0, 0, 0, 0.3)"),
    #     fillcolor="rgba(0, 0, 128, 0.2)",
    #     name="Thrower Robot"
    # )
    fig.add_trace(go.Scatter(x=[None], y=[None], mode='markers',
                             marker=dict(size=20,color="rgba(128, 128, 128, 0.5)"),  name="Reachable Range"))


    fig.add_trace(go.Scatter(x=ball_fail[:, 0], y=ball_fail[:, 1], mode='markers', name='Fail',
                             marker=dict(size=20, color='red', symbol='triangle-up', )))
    fig.add_trace(go.Scatter(x=ball_almost[:, 0], y=ball_almost[:, 1], mode='markers', name='Hit the Head',
                             marker=dict(size=20, color='yellow', symbol='square', )))
    fig.add_trace(go.Scatter(x=ball_success[:, 0], y=ball_success[:, 1], mode='markers', name='Successful Catch',
                             marker=dict(size=20, color='green', symbol='circle', )))

    fig.update_layout(
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(0,0,0,0)',
        xaxis_title='X',
        yaxis_title='Y',
        xaxis=dict(
            title='X(m)',
            title_font=dict(size=25),
            tickfont=dict(size=25),
            scaleanchor = 'y',
            scaleratio = 1,
            linewidth = 2,
            linecolor = 'black',
            showgrid=False,
        ),
        yaxis=dict(
            title='Y(m)',
            title_font=dict(size=25),
            tickfont=dict(size=25),
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        width=1000,
        height=1000,
        showlegend = True,
        legend=dict(
            font=dict(size=25),
            title_font=dict(size=25),
            orientation = "h",
            yanchor = "bottom",
            y = 0.7,  # 可以根据需要调整此值来放置图例
            xanchor = "center",
            x = 0.8
            ),
    )
    fig.show()
def plot_robot_move(saved_ball_data,saved_robot_data,success_df,parabola_filter):
    categories=[]
    move_distance=[]
    for i in range(0, 100):
        robot_s=[saved_robot_data[i][0][1],saved_robot_data[i][0][2]]
        robot_e=[saved_robot_data[i][-1][1],saved_robot_data[i][-1][2]]
        distance = math.sqrt((robot_s[0]-robot_e[0])**2 + (robot_s[1]-robot_e[1])**2)
        move_distance.append(distance)
        # y_local_new = y_local
        # x_local_new = x_local
        if success_df.iat[i, 1] == "s":
            categories.append("Successful Catch")
        elif success_df.iat[i, 1] == "a":
            categories.append("Hit the Head")
        elif success_df.iat[i, 1] == "f":
            categories.append("Fail")
    fig = go.Figure()
    # 对每个类别进行操作，并为每个类别添加直方图
    colors = ['red', 'orange', 'green']  # 指定三个类别的颜色
    category_names = ['Fail', 'Hit the Head','Successful Catch']  # 类别名称

    for category_name, color in zip(category_names, colors):
        fig.add_trace(go.Histogram(
            x=[move_distance[i] for i in range(len(move_distance)) if categories[i] == category_name],
            marker=dict(line=dict(width=2, color="black")),
            name=category_name,
            marker_color=color,
            nbinsx = 5
        ))
    # 设置布局
    fig.update_layout(
        bargap=0,
        barmode='stack',
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(0,0,0,0)',
        xaxis_title='Distance to Catching Robot(m)',
        yaxis_title='Frequency',
        xaxis=dict(
            # title='X(m)',
            title_font=dict(size=28),
            tickfont=dict(size=28),
            # scaleanchor='y',
            # scaleratio=1,
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        yaxis=dict(
            # title='Y(m)',
            title_font=dict(size=28),
            tickfont=dict(size=28),
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        width=500,
        height=700,
        showlegend=True,
        legend=dict(
            font=dict(size=25),
            title_font=dict(size=25),
            orientation = "h",
            yanchor = "bottom",
            y = 0.8,  # 可以根据需要调整此值来放置图例
            xanchor = "center",
            x = 1
        ),
    )
    fig.show()

def plot_throwing(saved_ball_data,saved_robot_data,success_df,parabola_filter):
    move_distance=[]
    for i in range(0, 100):
        robot_s=[saved_robot_data[i][0][1],saved_robot_data[i][0][2]]
        robot_e=[saved_robot_data[i][-1][1],saved_robot_data[i][-1][2]]
        distance = math.sqrt((robot_s[0]-robot_e[0])**2 + (robot_s[1]-robot_e[1])**2)
        move_distance.append(distance)
    fig = go.Figure()

    fig.add_trace(go.Histogram(
        x=move_distance,
        marker=dict(line=dict(width=2, color="black")),

        nbinsx = 20,
        xbins = dict(
            start=0,  # bin的起始值
            end=0.6,  # bin的结束值
            size=0.1  # 每个bin的宽度
        )
    ))
    # 设置布局
    fig.update_layout(
        bargap=0,
        barmode='stack',
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(0,0,0,0)',
        xaxis_title='Distance (m)',
        yaxis_title='Frequency',
        xaxis=dict(
            # title='X(m)',
            title_font=dict(size=28),
            tickfont=dict(size=28),
            # scaleanchor='y',
            # scaleratio=1,
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        yaxis=dict(
            # title='Y(m)',
            title_font=dict(size=28),
            tickfont=dict(size=28),
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        width=500,
        height=700,
        # showlegend=True,
        # legend=dict(
        #     font=dict(size=25),
        #     title_font=dict(size=25),
        #     orientation = "h",
        #     yanchor = "bottom",
        #     y = 0.8,  # 可以根据需要调整此值来放置图例
        #     xanchor = "center",
        #     x = 1
        # ),
    )
    fig.show()
def plot_landing(saved_ball_data,saved_robot_data,success_df,parabola_filter,model):

    parabola_filter = parabola_filter
    ball_start = []
    ball_end = []
    success_case = []
    ball_landing = []
    distance_list=[]
    for i in range(0, 100):

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
        ball_start.append([x1, y1])
        ball_end.append([x2, y2])
        success_case.append(success_df.iat[i, 1])
        rotation = -saved_robot_data[i][-1][4]
        # print(saved_robot_data[i])
        x_local, y_local = x2 - saved_robot_data[i][-1][1], y2 - saved_robot_data[i][-1][2]
        y_local_new = x_local * math.cos(rotation) + y_local * math.sin(rotation)
        x_local_new = -x_local * math.sin(rotation) + y_local * math.cos(rotation)
        ball_landing.append([x_local_new, y_local_new])

        robot_s=[saved_robot_data[i][0][1],saved_robot_data[i][0][2],saved_robot_data[i][0][4]]
        robot_self_pose = robot_s[2]
        # print(robot_self_pose)
        landing_target_x, landing_target_y, drop_t = landing_point_predictor_lstm(ball_data[:30], model,
                                                                                  robot_s,
                                                                                 0.3)
        print(landing_target_x,landing_target_y,x_local_new,y_local_new)
        distance=math.sqrt((landing_target_x-x_local)**2 + (landing_target_y-y_local)**2)
        distance_list.append(distance)
    fig = go.Figure()

    fig.add_trace(go.Histogram(
        x=distance_list,
        marker=dict(line=dict(width=2, color="black")),
        nbinsx=20,
        xbins=dict(
            start=0,  # bin的起始值
            end=0.6,  # bin的结束值
            size=0.1  # 每个bin的宽度
        )
    ))
    # 设置布局
    fig.update_layout(
        bargap=0,
        barmode='stack',
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(0,0,0,0)',
        xaxis_title='Distance (m)',
        yaxis_title='Frequency',
        xaxis=dict(
            # title='X(m)',
            title_font=dict(size=28),
            tickfont=dict(size=28),
            # scaleanchor='y',
            # scaleratio=1,
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        yaxis=dict(
            # title='Y(m)',
            title_font=dict(size=28),
            tickfont=dict(size=28),
            linewidth=2,
            linecolor='black',
            showgrid=False,
        ),
        width=500,
        height=700,
    )
    fig.show()
parabola_filter=ParabolaFitterRansac(200)
model = DynamicLSTM(input_dim=2, hidden_dim=20, output_dim=1,
                         num_layers=2)  # Initialize the same model structure
# plot_ball_robot_global(saved_ball_data,saved_robot_data,success_df,parabola_filter=parabola_filter)
plot_ball_robot_local(saved_ball_data,saved_robot_data,success_df,parabola_filter=parabola_filter)
# plot_robot_move(saved_ball_data,saved_robot_data,success_df,parabola_filter=parabola_filter)
# plot_throwing(saved_ball_data,saved_robot_data,success_df,parabola_filter=parabola_filter)
# plot_landing(saved_ball_data,saved_robot_data,success_df,parabola_filter,model)
# def cal_ball_landing(saved_ball_data):
#
#     x_data=[]
#     y_data=[]
#     x_land=[]
#     y_land=[]
#     parabola_filter=ParabolaFitterDirect3D(200)
#     for i in range(0,len(saved_ball_data)):
#         print(i)
#         # if i==27 or i==30:
#         #     continue
#         # print(saved_ball_data[i])
#
#         ball_data=saved_ball_data[i]
#         parabola_filter.fit(ball_data)
#         s1,s2=parabola_filter.solve(0.3)
#         x1=float(s1[0])
#         y1=float(s1[1])
#         x2=float(s2[0])
#         y2=float(s2[1])
#         if y1>0:
#             x1,x2=x2,x1
#             y1,y2=y2,y1
#         new_x=x2-x1
#         new_y=y2-y1
#         x_land.append(new_x)
#         y_land.append(new_y)
#         # print(i,s1,s2)
#         # x_data.append(float(s1[0]))
#         # y_data.append(float(s1[1]))
#         # x_data.append(float(s2[0]))
#         # y_data.append(float(s2[1]))
#     points=[x_land,y_land]
#     points=np.array(points)
#     return points
# def cal_robot_distribution(saved_robot_data):
#     x_s=[]
#     y_s=[]
#     x_e = []
#     y_e = []
#     for i in range(1,len(saved_robot_data)):
#         print(len(saved_robot_data[i]),i)
#         # for j in range(len(saved_robot_data[i])-1,len(saved_robot_data[i])):
#         x_s.append(saved_robot_data[i][0][1])
#         y_s.append(saved_robot_data[i][0][2])
#         x_e.append(saved_robot_data[i][-1][1])
#         y_e.append(saved_robot_data[i][-1][2])
#     return [x_s,y_s],[x_e,y_e]
# ball_points=cal_ball_landing(saved_ball_data)
# plot_2D(ball_points,"ball")


# prarbola_fitter=ParabolaFitterLSTM(model_path="../save_model.pth",number_of_points=30)
# prarbola_fitter.fit(data)
# plot_3D(data,prarbola_fitter)
# print(prarbola_fitter.solve(0.3))
