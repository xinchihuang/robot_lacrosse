import numpy as np
import time
import can
from pcan_cybergear import CANMotorController

bus = can.interface.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
motor1 = CANMotorController(bus, motor_id=101, main_can_id=254)
motor2 = CANMotorController(bus, motor_id=102, main_can_id=254)
motor3 = CANMotorController(bus, motor_id=103, main_can_id=254)

angle1 = motor1.enable()[1]
angle2 = motor2.enable()[1]
print(motor2.enable()[3])
angle3 = motor3.enable()[1]

step = 150
target1 = np.deg2rad(35)
step_length1 = (target1 - angle1) / step

for i in range(step):
    motor1.send_motor_control_command(torque=0, target_angle=angle1 + i * step_length1, target_velocity=0, Kp=100, Kd=1)
    time.sleep(0.001)

time.sleep(1)

angle1 = motor1.enable()[1]
angle2 = motor2.enable()[1]
angle3 = motor3.enable()[1]

step = 150
target1 = np.deg2rad(-30)
step_length1 = (target1 - angle1) / step

for i in range(step):
    motor1.send_motor_control_command(torque=0, target_angle=angle1 + i * step_length1, target_velocity=0, Kp=100, Kd=1)
    time.sleep(0.001)

time.sleep(1)

angle1 = motor1.enable()[1]
angle2 = motor2.enable()[1]
angle3 = motor3.enable()[1]

step = 600
target1 = np.deg2rad(0)
step_length1 = (target1 - angle1) / step

for i in range(step):
    motor1.send_motor_control_command(torque=0, target_angle=angle1 + i * step_length1, target_velocity=0, Kp=100, Kd=1)
    time.sleep(0.001)

bus.shutdown()