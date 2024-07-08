import pygame
import time
from robomaster import robot

# 初始化Pygame
pygame.init()

# 设置窗口大小
screen = pygame.display.set_mode((400, 300))

# 初始化机器人
ep_robot = robot.Robot()
ep_robot.initialize(conn_type="rndis")
ep_chassis = ep_robot.chassis

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
        elif event.type == pygame.KEYUP:
            if event.key in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, pygame.K_q, pygame.K_e]:
                move_car('stop')

# 关闭机器人连接
ep_robot.close()

# 退出Pygame
pygame.quit()
