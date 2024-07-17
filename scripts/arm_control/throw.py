import time
import math
import can
from scripts.arm_control.pcan_cybergear import CANMotorController

def throw_ball(brake_angle=math.pi/6, torque=-9):
    brake_angle = brake_angle
    torque = torque
    current_angle, current_speed = motor2.enable()[1:3]
    while current_angle > brake_angle:
        current_angle, current_speed = motor2.send_motor_control_command(torque=torque, target_angle=0, target_velocity=0,
                                                                         Kp=0, Kd=0)[1:3]
    while current_speed < -0.3:
        current_angle, current_speed = motor2.send_motor_control_command(torque=0, target_angle=0, target_velocity=0,
                                                                         Kp=0, Kd=5)[1:3]
    step = 500
    target = 0
    step_length = (target - current_angle) / step
    for i in range(step):
        motor2.send_motor_control_command(torque=0, target_angle=current_angle + i * step_length, target_velocity=0,
                                          Kp=100, Kd=1)
        time.sleep(0.001)


def move_lower_arm(target=math.pi/2):
    start_angle = motor2.enable()[1]
    print("start_angle", start_angle)
    step = 500
    target = target
    step_length = (target - start_angle) / step
    print("moving lower arm")
    for i in range(step):
        motor2.send_motor_control_command(torque=0, target_angle=start_angle + i * step_length, target_velocity=0,
                                          Kp=100, Kd=1)
        time.sleep(0.001)
    print("end_angle", target)

def move_upper_arm(target=-math.pi/6):
    start_angle = motor1.enable()[1]
    print("start_angle", start_angle)
    step = 1000
    target = target
    step_length = (target - start_angle)/step
    print("moving upper arm")
    for i in range(step):
        motor1.send_motor_control_command(torque=0, target_angle=start_angle + i * step_length, target_velocity=0, Kp=100, Kd=1)
        time.sleep(0.001)
    print("end_angle", target)
def throw_a_ball(motor1, motor2):
    # move_lower_arm(target=math.pi/4)
    # move_lower_arm(target=math.pi/2)
    # move_upper_arm(target=-math.pi/12)
    move_upper_arm(target=0)
    # move_upper_arm(target=0)
    # time.sleep(0.5)
    throw_ball(brake_angle=math.pi/3)
    # move_upper_arm(target=0)
    move_lower_arm(target=math.pi/2)
if __name__ == "__main__":
    bus = can.interface.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
    motor1 = CANMotorController(bus, motor_id=101, main_can_id=254)
    motor2 = CANMotorController(bus, motor_id=102, main_can_id=254)
    motor3 = CANMotorController(bus, motor_id=103, main_can_id=254)
