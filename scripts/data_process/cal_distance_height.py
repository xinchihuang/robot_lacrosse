import os

import numpy as np
from scipy.optimize import least_squares
import plotly.graph_objects as go
from scipy.optimize import minimize
from sympy import symbols, Eq, solve

for i in range(0, 40):
    if os.path.isfile(f'../saved_ball_data/{i}.npy'):

        # 加载数据

        arm_data=np.load(f'../saved_arm_data/{i}.npy')
        desired_angle=arm_data[1]
        desired_speed=arm_data[2]
        data = np.load(f'../saved_ball_data/{i}.npy')


        def filter_points(points, threshold=0.1):
            """过滤与相邻点距离过大的点"""
            filtered_points = []  # Initialize as a list to collect points
            for i in range(1, len(points) - 1):
                dist_prev = np.linalg.norm(points[i] - points[i - 1])
                dist_next = np.linalg.norm(points[i] - points[i + 1])
                if dist_prev <= threshold or dist_next <= threshold:
                    filtered_points.append(points[i])
            return np.array(filtered_points)  # Convert list to NumPy array outside the loop

        data = filter_points(data)
        # data = filter_points(data)

        # 提取 x, y, z 坐标
        x = data[:, 0]
        y = data[:, 1]
        z = data[:, 2]

        # 定义拟合函数
        def fit_plane_and_parabola(params):
            a, b, c, d, e, f, g, h = params
            # 计算 y = gx + h
            y_est = g * x + h
            # 计算 z = ax^2 + b*x*y + c*y^2 + d*x + e*y + f
            z_est = a * x**2 + b * x * y_est + c * y_est**2 + d * x + e * y_est + f
            # 返回差的平方和
            return np.concatenate([(y - y_est), (z - z_est)])

        # 初始参数猜测
        initial_guess = [1, 1, 1, 1, 1, 1, 1, 1]

        # 最小二乘法求解
        result = least_squares(fit_plane_and_parabola, initial_guess)

        a, b, c, d, e, f, g, h = result.x

        # 打印结果参数
        # print("Optimized parameters [a, b, c, d, e, f, g, h]:")
        # print(result.x)

        # 生成拟合曲面的数据
        xx = np.linspace(min(x), max(x), 100)
        yy = g * xx + h
        zz = a * xx**2 + b * xx * yy + c * yy**2 + d * xx + e * yy + f

        # 绘图
        fig = go.Figure()

        # 添加原始数据点
        fig.add_trace(go.Scatter3d(x=x, y=y, z=z, mode='markers', name='Data Points'))

        # 添加拟合曲线
        fig.add_trace(go.Scatter3d(x=xx, y=yy, z=zz, mode='lines', name='Fitted Parabola'))

        # 更新图形布局
        fig.update_layout(title=f'{i}',
                          scene=dict(
                              xaxis_title='X axis',
                              yaxis_title='Y axis',
                              zaxis_title='Z axis'
                          ))
        # fig.show()


        # 定义目标函数z
        def objective(x):
            y = g * x[0] + h
            return -(a * x[0]**2 + b * x[0] * y + c * y**2 + d * x[0] + e * y + f)  # 取负号因为我们需要最大化z

        # 初始猜测
        x0 = [0]

        # 调用minimize函数进行优化
        result = minimize(objective, x0)

        # 输出结果
        if result.success:
            optimal_x = result.x[0]
            optimal_y = g * optimal_x + h
            optimal_z = -result.fun  # 移除之前加上的负号
            # print(f"最优x值: {optimal_x}")
            # print(f"对应的y值: {optimal_y}")
            # print(f"最大的z值: {optimal_z}")
        else:
            print("优化失败")


        # 定义符号变量
        x, y = symbols('x y')

        # 使用得到的参数定义方程
        y_eq = Eq(y, g*x + h)
        z_eq = Eq(a*x**2 + b*x*y + c*y**2 + d*x + e*y + f, 0.3)

        # 将y方程代入z方程
        z_eq_substituted = z_eq.subs(y, g*x + h)

        # 求解x
        solution = solve(z_eq_substituted, x)

        # print("二次方程的两个根是：")
        # print(solution)
        # x1,y1 distance to x2,y2
        x1 = solution[0]
        x2 = solution[1]

        distance =abs(np.sqrt(g**2 + 1)*(x1 - x2))
        # print(f"两个根的距离是：{distance}")

        print(f"{i}\t{desired_angle}\t{desired_speed}\t{optimal_z}\t{distance}")
