import pygame
from robomaster import robot
import math
import time
import can
from pcan_cybergear import CANMotorController

# 初始化Pygame
pygame.init()

# 设置窗口大小
screen = pygame.display.set_mode((400, 300))

# 初始化机器人
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="rndis")
ep_chassis = ep_robot.chassis

# 初始化机械臂
bus = can.interface.Bus(bustype="socketcan", channel="can0", bitrate=1000000)
motor1 = CANMotorController(bus, motor_id=101, main_can_id=254)
motor2 = CANMotorController(bus, motor_id=102, main_can_id=254)
motor3 = CANMotorController(bus, motor_id=103, main_can_id=254)

motor1.set_0_pos()
motor2.set_0_pos()
motor3.set_0_pos()

motor1.enable()
motor2.enable()
motor3.enable()

# lock motor1 and motor3
motor1.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=100, Kd=1)
motor2.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=100, Kd=1)
motor3.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=100, Kd=1)

# 定义控制函数
def move_car(direction):
    x_val = 0.5
    y_val = 0.3
    z_val = 30

    if direction == 'w':
        ep_chassis.drive_speed(x=x_val, y=0, z=0, timeout=5)
    elif direction == 's':
        ep_chassis.drive_speed(x=-x_val, y=0, z=0, timeout=5)
    elif direction == 'a':
        ep_chassis.drive_speed(x=0, y=-y_val, z=0, timeout=5)
    elif direction == 'd':
        ep_chassis.drive_speed(x=0, y=y_val, z=0, timeout=5)
    elif direction == 'q':
        ep_chassis.drive_speed(x=0, y=0, z=-z_val, timeout=5)
    elif direction == 'e':
        ep_chassis.drive_speed(x=0, y=0, z=z_val, timeout=5)
    elif direction == 'stop':
        ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        time.sleep(0.1)  # 停止后稍微等待以确保机器人完全停止

def throw_ball():
    current_angle, current_speed = motor2.enable()[1:3]
    while current_angle>0.8:
        current_angle, current_speed = motor2.send_motor_control_command(torque=-1.5, target_angle=0, target_velocity=0, Kp=0, Kd=0)[1:3]
    while current_speed<-0.3:
        current_angle, current_speed = motor2.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=0, Kd=5)[1:3]
    step=1000
    target=0
    step_length=(target-current_angle)/step
    for i in range(step):
        motor2.send_motor_control_command(torque=0, target_angle=current_angle+i*step_length, target_velocity=0, Kp=100, Kd=1)
        time.sleep(0.001)

def release_arm():
    start_angle = motor2.enable()[1]
    step = 1000
    target=1.55
    step_length=(target-start_angle)/step
    for i in range(step):
        motor2.send_motor_control_command(torque=0, target_angle=start_angle+i*step_length, target_velocity=0, Kp=100, Kd=1)
        time.sleep(0.001)


# 主循环
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                move_car('w')
            elif event.key == pygame.K_s:
                move_car('s')
            elif event.key == pygame.K_a:
                move_car('a')
            elif event.key == pygame.K_d:
                move_car('d')
            elif event.key == pygame.K_q:
                move_car('q')
            elif event.key == pygame.K_e:
                move_car('e')
            elif event.key == pygame.K_1:
                release_arm()
            elif event.key == pygame.K_2:
                throw_ball()
        elif event.type == pygame.KEYUP:
            if event.key in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, pygame.K_q, pygame.K_e]:
                move_car('stop')
# 机械臂回到初始位置
throw_ball()

# 关闭机器人连接
ep_robot.close()

# 退出Pygame
pygame.quit()
