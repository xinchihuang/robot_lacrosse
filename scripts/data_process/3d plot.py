import pandas as pd
import plotly.graph_objects as go

# 加载数据
df = pd.read_csv('../diuqiubiao.csv',encoding='utf-16')  # 修改为你的文件路径
print(df)
# 创建3D图形
fig = go.Figure()
z=df['height']
# 添加抛球高度的3D曲面
fig = go.Figure(data=[go.Scatter3d(
    z=df['speed'],
    x=df['distance'],
    y=df['height'],
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
        zaxis=dict(range=[18, 20], title='speed'),
        xaxis=dict(range=[1, 3], title='distance'),
        yaxis=dict(range=[1.5, 1.8], title='height'),
    )
)
# 显示图形
fig.show()

# 添加抛球距离的3D曲面
fig = go.Figure(data=[go.Scatter3d(
    z=df['angle'],
    x=df['distance'],
    y=df['height'],
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
        zaxis=dict(range=[5, 10], title='angle'),
        xaxis=dict(range=[1, 3], title='distance'),
        yaxis=dict(range=[1.5, 1.8], title='height'),
    )
)

fig.show()
