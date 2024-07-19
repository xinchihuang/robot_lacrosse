import pandas as pd
import plotly.graph_objects as go

# 加载数据
df = pd.read_csv('../throw_data.csv')  # 修改为你的文件路径
print(df)
# 创建3D图形
fig = go.Figure()
z=df['抛球高度']
# 添加抛球高度的3D曲面
fig = go.Figure(data=[go.Scatter3d(
    z=df['刹车角度'],
    x=df['抛球距离'],
    y=df['抛球高度'],
    mode='markers',
    marker=dict(
        size=5,
        color=z,  # 颜色映射到Z轴的值
        colorscale='Viridis',  # 颜色范围
        opacity=0.8
    )
)])

# 更新图表布局，调整摄像机视角
fig.update_layout(
    title='抛球性能分析',
    autosize=True,
    scene=dict(
        zaxis=dict(range=[55, 65], title='刹车角度'),
        xaxis=dict(range=[1, 3], title='抛球距离'),
        yaxis=dict(range=[1.5, 1.8], title='抛球高度'),
    )
)
# 显示图形
fig.show()

# 添加抛球距离的3D曲面
fig = go.Figure(data=[go.Scatter3d(
    z=df['扭矩'],
    x=df['抛球距离'],
    y=df['抛球高度'],
    mode='markers',
    marker=dict(
        size=5,
        color=z,  # 颜色映射到Z轴的值
        colorscale='Viridis',  # 颜色范围
        opacity=0.8
    )
)])
# 更新图表布局，调整摄像机视角
fig.update_layout(
    title='抛球性能分析',
    autosize=True,
    scene=dict(
        zaxis=dict(range=[-12, -10], title='扭矩'),
        xaxis=dict(range=[1, 3], title='抛球距离'),
        yaxis=dict(range=[1.5, 1.8], title='抛球高度'),
    )
)

fig.show()
