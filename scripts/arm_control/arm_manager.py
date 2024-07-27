import math
import time
import can
import numpy as np
import plotly.express as px
import pandas as pd
from scripts.arm_control.pcan_cybergear import CANMotorController


class Arm:
    def __init__(self):
        self.bus = can.interface.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
        self.motor1 = CANMotorController(self.bus, motor_id=101, main_can_id=254)
        self.motor2 = CANMotorController(self.bus, motor_id=102, main_can_id=254)
        self.motor3 = CANMotorController(self.bus, motor_id=103, main_can_id=254)
        self.motor1_angle = 0
        self.motor2_angle = 0
        self.motor3_angle = 0
        self.enable_motors()

    def enable_motors(self):
        self.motor1_angle = self.motor1.set_0_pos()[1]
        self.motor2_angle = self.motor2.set_0_pos()[1]
        self.motor3_angle = self.motor3.set_0_pos()[1]
        self.motor1.enable()
        self.motor2.enable()
        self.motor3.enable()

        self.motor1.send_motor_control_command(torque=0,target_angle=self.motor1_angle, target_velocity=0,
                                                                   Kp=100, Kd=1)
        self.motor2.send_motor_control_command(torque=0, target_angle=self.motor2_angle, target_velocity=0,
                                                                   Kp=100, Kd=1)
        self.motor3.send_motor_control_command(torque=0,target_angle=self.motor3_angle, target_velocity=0,
                                                                   Kp=100, Kd=1)

    def go_to(self, control_list):
        """

        Args:
            control_list: [[step_number, motor1_target_angle, motor2_target_angle, motor3_target_angle], [step_number, motor1_target_angle, motor2_target_angle, motor3_target_angle]]

        Returns:

        """
        for control in control_list:
            move_step = control[0]
            motor1_target_angle = np.deg2rad(control[1])
            motor2_target_angle = np.deg2rad(control[2])
            motor3_target_angle = np.deg2rad(control[3])
            motor1_start_angle = self.motor1.enable()[1]
            motor2_start_angle = self.motor2.enable()[1]
            motor3_start_angle = self.motor3.enable()[1]
            motor1_step_angle = (motor1_target_angle - motor1_start_angle) / move_step
            motor2_step_angle = (motor2_target_angle - motor2_start_angle) / move_step
            motor3_step_angle = (motor3_target_angle - motor3_start_angle) / move_step
            for i in range(move_step):
                self.motor1_angle = self.motor1.send_motor_control_command(torque=0,
                                                                           target_angle=motor1_start_angle + motor1_step_angle * (
                                                                                   i + 1), target_velocity=0,
                                                                           Kp=100, Kd=1)[1]
                self.motor2_angle = self.motor2.send_motor_control_command(torque=0,
                                                                           target_angle=motor2_start_angle + motor2_step_angle * (
                                                                                   i + 1), target_velocity=0,
                                                                           Kp=100, Kd=1)[1]
                self.motor3_angle = self.motor3.send_motor_control_command(torque=0,
                                                                           target_angle=motor3_start_angle + motor3_step_angle * (
                                                                                   i + 1), target_velocity=0,
                                                                           Kp=100, Kd=1)[1]
            time.sleep(1)

    def throw_to_angle_with_torque(self, brake_angle=np.deg2rad(45), torque=-12):
        start_time = time.time()
        record_list = []
        brake_angle = np.deg2rad(brake_angle)
        torque = torque
        current_angle, current_speed = self.motor2.enable()[1:3]
        # print("Current angle: ", current_angle)
        # print("Current speed: ", current_speed)
        while current_angle > brake_angle:
            # print("acceleration")
            current_angle, current_speed = self.motor2.send_motor_control_command(torque=torque, target_angle=0,
                                                                                  target_velocity=0,
                                                                                  Kp=0, Kd=0)[1:3]
            time_elapsed = time.time() - start_time
            record_list.append([time_elapsed, current_angle, current_speed,0])
        while current_speed < -0.3:
            # print("deceleration")
            current_angle, current_speed = self.motor2.send_motor_control_command(torque=0, target_angle=0,
                                                                                  target_velocity=0,
                                                                                  Kp=0, Kd=5)[1:3]
            time_elapsed = time.time() - start_time
            record_list.append([time_elapsed, current_angle, current_speed,1])
        step = 500
        target = 0
        step_length = (target - current_angle) / step
        # print("to 0")
        for i in range(step):
            self.motor2.send_motor_control_command(torque=0, target_angle=current_angle + i * step_length,
                                                   target_velocity=0,
                                                   Kp=100, Kd=1)
            time.sleep(0.001)
        # print("done")
        self.motor2_angle = 0
        return record_list
    time.sleep(1)

    def throw_to_angle_with_speed(self, target_angle=45, target_speed=-20,lower_angle=0,upper_angle=90,lower_speed=0,upper_speed=30):
        """

        Args:
            target_angle: degree from parallel to ground
            target_speed: rad/s front is positive
            lower_angle: limit of lower angle degree
            upper_angle: limit of upper angle degree
            lower_speed: limit of lower speed rad/s
            upper_speed: limit of upper speed rad/s

        Returns:

        """
        start_time = time.time()
        record_list = []
        target_angle=90 - target_angle
        if target_angle < lower_angle:
            target_angle = lower_angle
        elif target_angle> target_angle:
            target_angle=upper_angle

        target_speed = -target_speed
        if target_speed < lower_speed:
            target_speed = lower_speed
        elif target_speed> target_speed:
            target_speed=upper_speed


        target_rad = np.deg2rad(target_angle)
        # current_rad, current_speed = self.motor2.enable()[1:3]
        # print("Current angle: ", current_angle)
        # print("Current speed: ", current_speed)
        while True:
            # print("acceleration")
            current_rad, current_speed = self.motor2.send_motor_control_command(torque=0, target_angle=target_rad,
                                                                                  target_velocity=target_speed,
                                                                                  Kp=100, Kd=1)[1:3]
            time_elapsed = time.time() - start_time
            record_list.append([time_elapsed, current_rad, current_speed, 0])
            if time_elapsed >2:
                break
        time.sleep(1)
        current_rad, current_speed = self.motor2.send_motor_control_command(torque=0, target_angle=target_rad,
                                                                            target_velocity=target_speed,
                                                                            Kp=100, Kd=1)[1:3]
        step = 500
        target = np.deg2rad(90)
        step_length = (target - current_rad) / step
        # print("to parallel to ground")
        for i in range(step):
            self.motor2.send_motor_control_command(torque=0, target_angle=current_rad + i * step_length,
                                                   target_velocity=0,
                                                   Kp=100, Kd=1)
            time.sleep(0.001)
        # print("done")
        self.motor2_angle = 0
        return record_list

    def keep(self):
        self.motor1.send_motor_control_command(torque=0, target_angle=self.motor1_angle, target_velocity=0, Kp=100,
                                               Kd=1)
        self.motor2.send_motor_control_command(torque=0, target_angle=self.motor2_angle, target_velocity=0, Kp=100,
                                               Kd=1)
        self.motor3.send_motor_control_command(torque=0, target_angle=self.motor3_angle, target_velocity=0, Kp=100,
                                               Kd=1)
    def stop(self):
        self.motor1.disable()
        self.motor2.disable()
        self.motor3.disable()




# Convert the data to a DataFrame
# df = pd.DataFrame(data, columns=['time', 'position', 'speed', 'status'])
#
# # Create a scatter plot for position vs time
# fig1 = px.scatter(df, x='time', y='position', color='status', title='Time vs Position Plot')
#
# # Create a scatter plot for speed vs time
# fig2 = px.scatter(df, x='time', y='speed', color='status', title='Time vs Speed Plot')
#
# # Show the plots
# fig1.show()
# fig2.show()
