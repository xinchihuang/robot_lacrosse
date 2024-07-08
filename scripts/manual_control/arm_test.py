import math
import time
import can
from pcan_cybergear import CANMotorController

bus = can.interface.Bus(bustype="socketcan", channel="can0", bitrate=1000000)
motor1 = CANMotorController(bus, motor_id=101, main_can_id=254)
motor2 = CANMotorController(bus, motor_id=102, main_can_id=254)
motor3 = CANMotorController(bus, motor_id=103, main_can_id=254)

def throw_ball():
    current_angle, current_speed = motor2.enable()[1:3]
    while current_angle>-1:
        current_angle, current_speed = motor2.send_motor_control_command(torque=-9, target_angle=0, target_velocity=0, Kp=0, Kd=0)[1:3]
    while current_speed<-0.3:
        current_angle, current_speed = motor2.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=0, Kd=5)[1:3]
    step=1000
    target=-2.5
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
        
print(motor1.set_0_pos())
print(motor2.set_0_pos())
print(motor3.set_0_pos())
print(motor1.enable())
print(motor2.enable())
print(motor3.enable())
print(motor1.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=100, Kd=1))
print(motor2.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=100, Kd=1))
print(motor3.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=100, Kd=1))
#for i in range(15):
    #print(motor1.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=100, Kd=1))
    #print(motor2.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=100, Kd=1))
    #print(motor3.send_motor_control_command(torque=0, target_angle=0, target_velocity=0, Kp=100, Kd=1))
    #time.sleep(1)


time.sleep(2)

throw_ball()

time.sleep(2)

print(motor1.disable())
print(motor2.disable())
print(motor3.disable())


