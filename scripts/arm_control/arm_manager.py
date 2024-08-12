import math
import time
import can
import numpy as np

from scripts.arm_control.pcan_cybergear import CANMotorController


class Arm:
    def __init__(self):
        self.bus = can.interface.Bus(interface="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
        self.motor1 = CANMotorController(self.bus, motor_id=101, main_can_id=254)
        self.motor2 = CANMotorController(self.bus, motor_id=102, main_can_id=254)
        # self.motor3 = CANMotorController(self.bus, motor_id=103, main_can_id=254)
        self.motor1_angle = 0
        self.motor2_angle = 0
        # self.motor3_angle = 0
    def enable_motors(self):
        """
        Enable motors, please direct arm to sky after it sound release it. Arm will lay down to initial position (link 2 lay down)
        """
        self.motor1.enable()
        self.motor2.enable()
        # self.motor3.enable()
        self.motor1.set_0_pos()
        self.motor2.set_0_pos()
        # self.motor3.set_0_pos()
        self.motor1.send_motor_control_command(torque=0,target_angle=self.motor1_angle, target_velocity=0,
                                                                   Kp=100, Kd=1)
        self.motor2.send_motor_control_command(torque=0, target_angle=self.motor2_angle, target_velocity=0,
                                                                   Kp=100, Kd=1)
        # self.motor3.send_motor_control_command(torque=0,target_angle=self.motor3_angle, target_velocity=0,
        #                                                            Kp=100, Kd=1)
        # warning to release arm
        for i in range(6):
            print("release arm motor, count down: " + str(5-i))
            time.sleep(1)
        # lay down link2 to 90 degree
        step = 500
        target = np.deg2rad(90)
        step_length = target / step
        # print("to parallel to ground")
        for i in range(step):
            self.motor2.send_motor_control_command(torque=0, target_angle= i * step_length,
                                                   target_velocity=0,
                                                   Kp=100, Kd=1)
            time.sleep(0.001)
        # print("done")
        self.motor1_angle = 0
        self.motor2_angle = np.deg2rad(90)
        # self.motor3_angle = 0
    def reset_ball(self):
        angle1 = self.motor1.enable()[1]
        angle2 = self.motor2.enable()[1]
        # print(self.motor2.enable()[3])
        # angle3 = self.motor3.enable()[1]

        step = 150
        target1 = np.deg2rad(35)
        step_length1 = (target1 - angle1) / step

        for i in range(step):
            self.motor1.send_motor_control_command(torque=0, target_angle=angle1 + i * step_length1, target_velocity=0, Kp=100, Kd=1)
            time.sleep(0.001)

        time.sleep(1)

        angle1 = self.motor1.enable()[1]
        angle2 = self.motor2.enable()[1]
        # angle3 = self.motor3.enable()[1]

        step = 150
        target1 = np.deg2rad(-30)
        step_length1 = (target1 - angle1) / step

        for i in range(step):
            self.motor1.send_motor_control_command(torque=0, target_angle=angle1 + i * step_length1, target_velocity=0, Kp=100, Kd=1)
            time.sleep(0.001)

        time.sleep(1)

        angle1 = self.motor1.enable()[1]
        angle2 = self.motor2.enable()[1]
        # angle3 = self.motor3.enable()[1]

        step = 600
        target1 = np.deg2rad(0)
        step_length1 = (target1 - angle1) / step

        for i in range(step):
            self.motor1.send_motor_control_command(torque=0, target_angle=angle1 + i * step_length1, target_velocity=0, Kp=100, Kd=1)
            time.sleep(0.001)


    def throw_to_angle_with_speed(self, target_angle=45, target_speed=20,lower_angle=0,upper_angle=90,lower_speed=-30,upper_speed=0):
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
        record_list = []
        target_angle=90 - target_angle
        if target_angle < lower_angle:
            target_angle = lower_angle
        elif target_angle> target_angle:
            target_angle=upper_angle
        target_rad = np.deg2rad(target_angle) # if given 30deg, target_rad will be pi/3

        target_speed = -target_speed
        if target_speed < lower_speed:
            target_speed = lower_speed
        elif target_speed> target_speed:
            target_speed=upper_speed
	
	    # link2 lay down down
        current_rad, current_speed = self.motor2.enable()[1:3]
        step = 500
        target = np.deg2rad(91)
        step_length = (target - current_rad) / step
        # print("to down down")
        for i in range(step):
            self.motor2.send_motor_control_command(torque=0, target_angle=current_rad + i * step_length,
                                                   target_velocity=0,
                                                   Kp=100, Kd=1)
            time.sleep(0.001)
        # current_rad, current_speed = self.motor2.enable()[1:3]
        # print("Current angle: ", current_angle)
        # print("Current speed: ", current_speed)
	time.sleep(0.1)
        start_time = time.time()
        while True:
            # print("acceleration")
            current_rad, current_speed, current_tor = self.motor2.send_motor_control_command(torque=0, target_angle=target_rad,
                                                                                  target_velocity=target_speed,
                                                                                  Kp=100, Kd=1)[1:4]
            time_elapsed = time.time() - start_time
            record_list.append([round(time_elapsed,8), round(current_rad,8), round(current_speed,8), round(current_tor,8)])
            if time_elapsed >0.25:
                break
        time.sleep(1.75)
        current_rad, current_speed = self.motor2.send_motor_control_command(torque=0, target_angle=target_rad,
                                                                            target_velocity=target_speed,
                                                                            Kp=100, Kd=1)[1:3]
        # link2 lay down
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
        self.motor2_angle = np.deg2rad(90)
        return record_list
    def stop(self):
        self.motor1.disable()
        self.motor2.disable()
        # self.motor3.disable()


class Launcher:
    def __init__(self):
        self.bus =can.interface.Bus(interface="socketcan", channel="can0", bitrate=1000000)
        self.motor1 = CANMotorController(self.bus, motor_id=101, main_can_id=254)
        # self.motor3 = CANMotorController(self.bus, motor_id=103, main_can_id=254)
        self.motor1_angle = 0

    def move_at_acceleration(self, target_angle=37, target_velocity=15):
        '''
        :param target_angle: rad
        :param target_velocity: rad/s
        :return: None
        '''
        self.motor1.set_0_pos()
        # self.motor1.enable()
        target_angle = np.deg2rad(target_angle)
        start_time = time.time()
        current_angle = self.motor1.enable()[1]
        acceleration = target_velocity ** 2 / (2 * (target_angle - current_angle))
        print(f'acceleration: {acceleration}')
        time_to_accelerate = target_velocity / acceleration
        print(f'time_to_accelerate: {time_to_accelerate}')
        total_time = time_to_accelerate * 2
        print(f'total_time: {total_time}')
        while time.time() - start_time < time_to_accelerate:
            current_time = time.time() - start_time
            # record_time.append(current_time)
            current_velocity = acceleration * current_time
            current_angle = current_velocity * current_time / 2
            angle, speed = self.motor1.send_motor_control_command(torque=0, target_angle=current_angle,
                                                                  target_velocity=current_velocity, Kp=100, Kd=1)[1:3]
            # record_angle.append(angle)
            # record_velocity.append(speed)
            time.sleep(0.001)
        while time.time() - start_time < total_time:
            current_time = time.time() - start_time
            # record_time.append(current_time)
            current_velocity = target_velocity - acceleration * (current_time - time_to_accelerate)
            current_angle = target_angle + target_velocity * (current_time - time_to_accelerate) - acceleration * (
                        current_time - time_to_accelerate) ** 2 / 2
            angle, speed = self.motor1.send_motor_control_command(torque=0, target_angle=current_angle,
                                                                  target_velocity=current_velocity, Kp=100, Kd=1)[1:3]
            # record_angle.append(angle)
            # record_velocity.append(speed)
            time.sleep(0.001)

    def stop(self):
        self.motor1.disable()
        # self.motor2.disable()
        # self.motor3.disable()

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
