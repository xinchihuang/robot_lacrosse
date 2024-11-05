import plotly.graph_objects as go
import numpy as np
import pandas as pd
from scripts.chassis_control.parabola_predictor import *
import plotly.express as px
def plot_3D(points,predictor,name=""):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]
    y_f=[]
    z_f=[]
    x_f=[]
    sample_point_x = np.linspace(min(x), max(x), 100)
    sample_point_y = np.linspace(min(y), max(y), 100)
    for i in range(100):
        x_i,y_i,z_i=predictor.predict(sample_point_x[i],sample_point_y[i])
        x_f.append(x_i)
        y_f.append(y_i)
        z_f.append(z_i)
    # 绘图
    fig = go.Figure()
    print(len(x))
    index = np.arange(len(x))
    data = {'x': x, 'y': y, 'z': z, 'index': index}
    df = pd.DataFrame(data)


    # Create a 3D scatter plot with color based on index
    fig = px.scatter_3d(df, x='x', y='y', z='z', color='index', color_continuous_scale=px.colors.sequential.Bluered)

    # 添加拟合曲线
    fig.add_trace(go.Scatter3d(x=x_f, y=y_f, z=z_f, mode='lines', name='Fitted Parabola'))

    # 更新图形布局
    fig.update_layout(title=name,
                      scene=dict(
                          xaxis_title='X axis',
                          yaxis_title='Y axis',
                          zaxis_title='Z axis'
                      ))
    fig.show()

def plot_2D(points,name=""):
    x = points[0]
    print(type(x[0]))
    y = points[1]
    labels = [str(i) for i in range(1, len(x) )]
    fig = go.Figure()

    # 添加原始数据点
    fig.add_trace(go.Scatter(x=x, y=y, mode='markers+text', name='Data Points',
                            text=labels,  # 添加文本标签
                            textposition='top center',))

    # 如果需要添加拟合曲线，确保你有拟合后的数据 x_f 和 y_f
    # fig.add_trace(go.Scatter(x=x_f, y=y_f, mode='lines', name='Fitted Curve'))

    # 更新图形布局，适用于二维图形
    fig.update_layout(
        title=name,
        xaxis_title='X axis',
        yaxis_title='Y axis',
        # xaxis = dict(range=[-1.5, 1.5]),
        # yaxis = dict(range=[1, 4]),

        width=700,  # 设定画布宽度
        height=700
    )
    fig.show()

def plot_robot(robot_s,robot_e, name=""):
    x_s = robot_s[0]
    y_s = robot_s[1]
    x_e = robot_e[0]
    y_e = robot_e[1]
    labels_s = [str(i) for i in range(1, len(x_s))]
    labels_e = [str(i) for i in range(1, len(x_e))]
    fig = go.Figure()

    # 添加原始数据点
    fig.add_trace(go.Scatter(x=x_s, y=y_s, mode='markers+text', name='Data Points',
                             text=labels_s,  # 添加文本标签
                             textposition='top center', ))
    fig.add_trace(go.Scatter(x=x_e, y=y_e, mode='markers+text', name='Data Points',
                             text=labels_e,  # 添加文本标签
                             textposition='top center', ))

    # 如果需要添加拟合曲线，确保你有拟合后的数据 x_f 和 y_f
    # fig.add_trace(go.Scatter(x=x_f, y=y_f, mode='lines', name='Fitted Curve'))

    # 更新图形布局，适用于二维图形
    fig.update_layout(
        title=name,
        xaxis_title='X axis',
        yaxis_title='Y axis',
        # xaxis = dict(range=[-1.5, 1.5]),
        # yaxis = dict(range=[1, 4]),

        width=700,  # 设定画布宽度
        height=700
    )
    fig.show()
def filter_points(points, threshold=0.1):
    """过滤与相邻点距离过大的点"""
    filtered_points = []  # Initialize as a list to collect points
    for i in range(1, len(points) - 1):
        dist_prev = np.linalg.norm(points[i] - points[i - 1])
        dist_next = np.linalg.norm(points[i] - points[i + 1])
        if dist_prev <= threshold or dist_next <= threshold:
            filtered_points.append(points[i])
    return np.array(filtered_points)  # Convert list to NumPy array outside the loop
if __name__ == '__main__':
    for i in range(6,7):
        data = np.load(f'../saved_data/saved_ball_data/{i}.npy')
        data = filter_points(data)
        parabola_filter=ParabolaFitterRansac(300)
        parabola_filter.fit(data)
        plot_3D(data, parabola_filter,name=str(i))
