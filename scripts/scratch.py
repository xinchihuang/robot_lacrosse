import pybullet as p
import pybullet_data
import time

# 启动 PyBullet 仿真器
physicsClient = p.connect(p.GUI)

# 添加资源路径
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 设置重力
p.setGravity(0, 0, -10)

# 加载地面
planeId = p.loadURDF("plane.urdf")


ballId = p.loadURDF("/home/xinchi/catkin_ws/src/robomaster_arm_description/urdf/ball.urdf", basePosition=[0, 0, 1],
                    globalScaling=1.0,
                    physicsClientId=physicsClient,
                    useMaximalCoordinates=False,
                    flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL)
p.changeDynamics(ballId, -1, contactStiffness=5000000000, contactDamping=5000)
# 简单的仿真循环
while True:
    p.stepSimulation()
    time.sleep(1./240.)

# 断开连接
# p.disconnect()
