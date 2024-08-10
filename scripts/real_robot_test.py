import math
import random
import sys
from scripts.optitrack_sdk.NatNetClient import NatNetClient
from scripts.robomaster_executor.robomaster_executor import RoboMasterExecutor
from Robot import Robot
from scripts.arm_control.arm_executor import ArmExecutor
from scripts.utils import optitrack_coordinate_to_world_coordinates
from scripts.data_process.check_parabola_point import check_parabola_point
from threading import Thread
import os
import time
import numpy as np
def my_parse_args(arg_list, args_dict):
    # set up base values
    arg_list_len=len(arg_list)
    if arg_list_len>1:
        args_dict["serverAddress"] = arg_list[1]
        if arg_list_len>2:
            args_dict["clientAddress"] = arg_list[2]
        if arg_list_len>3:
            if len(arg_list[3]):
                args_dict["use_multicast"] = True
                if arg_list[3][0].upper() == "U":
                    args_dict["use_multicast"] = False

    return args_dict
def cal_angle_speed(h,d,g=9.8,arm_length=0.28):
    v_vertical=math.sqrt(2*g*h)
    t=math.sqrt(8*h/g)
    v_horizon=d/t
    linear_speed=math.sqrt(v_horizon**2+v_vertical**2)
    angle=math.atan(v_horizon/v_vertical)
    return math.degrees(angle),linear_speed/arm_length
def simple_moving_average(data, window_size=10):
    sma = []
    for i in range(len(data)):
        if i+1 >= window_size:
            sma.append(sum(data[i-window_size+1:i+1]) / window_size)
        else:
            sma.append(None)  # 表示这个位置还不能计算平均值
    return sma
def check_ball_memory_records(ball_memory,check_window=20):
    """
    Check whether to stop a recording session
    Args:
        ball_memory: record ball positions

    Returns:  bool (whether to stop the recording session)

    """
    if len(ball_memory)<check_window:
        return False
    else:
        sma=simple_moving_average(ball_memory)
        previous=float("inf")
        for i in range(len(sma)):
            if sma[i] is None:
                continue
            else:
                if sma[i]<previous:
                    previous=sma[i]
                else:
                    return False
        return True

class Experiment:
    def __init__(self):
        self.ball_memory=[]
        self.robot_list=[]

        self.check_point_window_size=30
        self.check_parabola_window_size = 20


        self.state="idle"

        ### throw_related
        self.throw_h=1.5
        self.g=9.8

        self.throw_starts_time=time.time()

        ##save_related
        self.saved=False
        self.saved_ball_data = []
        self.saved_robot_data = []
        self.saved_arm_input=[]
        self.saved_arm_record=[]

    def handle_command(self):
        """
        Handles commands name,state(catch,reset,throw),extra command(desired arm angle,desired speed)
        Returns:

        """
        while True:
            command=input("Press enter command: ")
            command_list=command.split(",")
            state=command_list[0]
            if state == "throw":

                if len(self.robot_list) == 1:
                    robot=self.robot_list[0]
                    robot.robot_state = "throw"
                    self.throw_h, distance=1.5,0.94
                    desired_angle, desired_speed = cal_angle_speed(self.throw_h, distance)

                    arm_msg = robot.arm_throw_ball(desired_angle, desired_speed)
                    # self.arm_msg=arm_msg.decode()
                    # print(self.arm_msg)
                    self.saved_arm_input = [1, desired_angle, desired_speed]

                elif len(self.robot_list) == 2:
                    thrower_name = command_list[1]
                    catcher_name = command_list[2]
                    for robot in self.robot_list:
                        if thrower_name==robot.robot_name:
                            robot.robot_state = "throw"
                        if catcher_name==robot.catcher_name:
                            robot.robot_state ="catcher"
                self.state="throw"
                self.throw_starts_time=time.time()
            elif state == "reset":
                self.state="reset"
                for robot in self.robot_list:
                    robot.robot_state = state
                    robot.reset_arm()
            elif state == "rotate":
                self.state="rotate"
                for robot in self.robot_list:
                    robot.robot_state = state
    def move_arm(self):
        while True:
            if self.state == "throw":
                if len(self.robot_list) == 1:
                    robot=self.robot_list[0]
                    if robot.state=="throw":
                        self.throw_h, distance=1.5,0.94
                        desired_angle, desired_speed = cal_angle_speed(self.throw_h, distance)
                        arm_msg = robot.arm_throw_ball(desired_angle, desired_speed)
                        # self.arm_msg=arm_msg.decode()
                        # print(self.arm_msg)
                        self.saved_arm_input = [1, desired_angle, desired_speed]

                else:
                    thrower=None
                    catcher=None
                    for robot in self.robot_list:
                        if robot.state=="throw":
                            thrower=robot
                        elif robot.state=="catch":
                            catcher=robot
                    if not thrower is None and not catcher is None:
                        distance = math.sqrt((thrower.robot_self_pose[0] - catcher.robot_self_pose[0]) ** 2 + (
                                    thrower.robot_self_pose[1] - catcher.robot_self_pose[1]) ** 2)
                        desired_angle, desired_speed = cal_angle_speed(self.throw_h, distance)
                        arm_msg = thrower.arm_throw_ball(desired_angle, desired_speed)
                        print(desired_angle, desired_speed, distance)
                        desired_speed = desired_speed - 10
                        self.saved_arm_input = [1, desired_angle, desired_speed]
            elif self.state == "reset":
                self.throwing = False
                for robot in self.robot_list:
                    robot.state = "reset"
                    robot.reset_arm()

        pass
    def move_robot(self):
        while True:
            if len(self.robot_list) == 2 and self.state=="rotate":
                robot1 = self.robot_list[0]
                robot2 = self.robot_list[1]
                if not robot1.robot_self_pose is None and not robot2.robot_self_pose:
                    if "1" == str(id):
                        vx, vy, omega = robot1.get_rotate_control(robot2.robot_self_pose)
                        robot1.execute(vx, vy, omega)
                    elif "2" == str(id):
                        vx, vy, omega = robot2.get_rotate_control(robot1.robot_self_pose)
                        robot2.execute(vx, vy, omega)
            elif self.state=="throw":
                for robot in self.robot_list:
                    if robot.robot_name == str(id):
                        if robot.robot_state== "catch":
                            vx, vy, omega = robot.get_move_cotrol(self.ball_memory[:self.check_point_window_size])
                            # print(vx,vy,omega)
                            robot.execute(vx, vy, omega)
            elif self.state=="idle":
                for robot in self.robot_list:
                    robot.robot_state="idle"

    def process_optitrack_rigid_body_data_old(self, id, position, rotation):
        """

        Args:
            id: rigid body id
            position: rigid body position
            rotation: rigid body rotation

        Returns:

        """

        # print(position)
        for robot in self.robot_list:
            if robot.robot_name == str(id):
                x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation)

                robot.robot_self_pose=[x_world, y_world, z_world, theta_world]
                if self.throwing == True:
                    self.saved_robot_data.append([id,x_world, y_world, z_world])
                if robot.state=="catch":
                    robot.ball_memory=self.ball_memory[10:self.check_point_window_size+10]
                    vx, vy, omega=robot.generate_cotrol(x_world, y_world, z_world, theta_world)
                    # print(vx,vy,omega)
                    robot.execute(vx, vy, omega)

        if len(self.robot_list) == 2:
            robot1 = self.robot_list[0]
            robot2 = self.robot_list[1]
            if not robot1.robot_self_pose is None and not robot2.robot_self_pose is None and robot1.state== "rotate" and  robot2.state== "rotate":
                # print(robot1.robot_self_pose, robot2.robot_self_pose)
                # print(robot1.robot_name, robot1.robot_self_pose)
                # print( robot2.robot_name, robot2.robot_self_pose)
                if "1" == str(id):
                    vx, vy, omega=robot1.rotate(robot1.robot_self_pose, robot2.robot_self_pose)
                    robot1.execute(vx, vy, omega)
                elif "2" == str(id):
                    vx, vy, omega=robot2.rotate(robot2.robot_self_pose, robot1.robot_self_pose)
                    robot2.execute(vx, vy, omega)

    def process_optitrack_rigid_body_data(self, id, position, rotation):
        """

        Args:
            id: rigid body id
            position: rigid body position
            rotation: rigid body rotation

        Returns:

        """
        # print(position)
        for robot in self.robot_list:
            if robot.robot_name == str(id):
                x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation)
                robot.robot_self_pose = [x_world, y_world, z_world, theta_world]
                if self.state == "throw":
                    self.saved_robot_data.append([id, x_world, y_world, z_world])

    def process_optitrack_ball_data_old(self, mocap_data):
        """

        Args:
            mocap_data: process ball data

        Returns: None

        """
        position = None
        rotation = None
        z_world = None

        ### filter out point that too low
        for marker_id in range(len(mocap_data.labeled_marker_data.labeled_marker_list)):
            if mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1] > 0.2:
                position = [mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[0],
                            mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1],
                            mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[2]]
                rotation = [0, 0, 0, 0]

        # print(position)
        if not position == None and self.throwing==True:
            x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation,is_ball=True)
            if z_world > 0.4 and x_world ** 2 + y_world ** 2 < 2.25:
                # present_time = time.time()
                self.ball_memory.append([x_world, y_world, z_world])
                # is_parabola = check_parabola_point(self.ball_memory)
                # print(is_parabola,len(self.ball_memory))
                # if not is_parabola and len(self.ball_memory) > self.check_parabola_window_size:
                #     self.ball_memory = []
                # print(z_world, len(self.ball_memory))
                if self.saved == False:
                    self.saved_ball_data.append([x_world, y_world, z_world])


            else:
                if len(self.saved_ball_data) > 50 and self.saved == False:
                   self.save_data()
                self.saved_robot_data=[]
                self.ball_memory = []

    def process_optitrack_ball_data(self, mocap_data):
        """

        Args:
            mocap_data: process ball data

        Returns: None

        """
        position = None
        rotation = None
        z_world = None
        if self.state=="throw" and time.time()-self.throw_starts_time<3:
            ### filter out point that too low
            for marker_id in range(len(mocap_data.labeled_marker_data.labeled_marker_list)):
                if mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1] > 0.2:
                    position = [mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[0],
                                mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1],
                                mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[2]]
                    rotation = [0, 0, 0, 0]
            if not position == None:
                x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation,                                                                                is_ball=True)
                self.ball_memory.append([x_world, y_world, z_world])
            else:
                stop=check_ball_memory_records(self.ball_memory)
                if stop==True:
                    self.ball_memory = []
                    self.state = "idle"
                else:
                    pass
        else:
            self.ball_memory = []
            self.state = "idle"
            pass


    def save_data(self):
        files_and_dirs = os.listdir(
            "saved_ball_data/")
        # Filter out directories, keeping only files
        files = [f for f in files_and_dirs if os.path.isfile(
            os.path.join("saved_ball_data/", f))]
        number = len(files)
        np.save("./saved_ball_data/" + str(
            number) + ".npy", np.array(self.saved_ball_data))
        np.save("./saved_robot_data/" + str(
            number) + ".npy", np.array(self.saved_robot_data))
        np.save("./saved_arm_data/" + str(
            number) + ".npy", np.array(self.saved_arm_input))
        # np.save("./saved_arm_data/" + str(
        #     number) + ".npy", np.array(self.saved_arm_input))
        print("saved " + str(number))
        self.saved_ball_data = []
        self.saved_robot_data = []
        self.saved = True
if __name__ == "__main__":
    # robot1_chassis_executor=RoboMasterExecutor(sn="3JKCH8800101C2")
    # robot1_arm_executor = ArmExecutor(('192.168.0.105', 12345))
    robot2_chassis_executor = RoboMasterExecutor(sn="3JKCH7T00100M9")
    robot2_arm_executor = ArmExecutor(('192.168.0.104', 12345))
    # ball_launcher = None
    # ball_launcher_arm_executor = ArmExecutor(('192.168.0.104', 12345))
    # robot1_chassis_executor=None
    # robot1_arm_executor = None
    # robot2_chassis_executor = None
    # robot2_arm_executor=None

    # robot1 = Robot('1', robot1_chassis_executor, robot1_arm_executor)
    robot2 = Robot('2', robot2_chassis_executor,robot2_arm_executor)
    experiment=Experiment()
    # experiment.robot_list.append(robot1)
    experiment.robot_list.append(robot2)

    optionsDict = {}
    optionsDict["clientAddress"] = "127.0.0.1"
    optionsDict["serverAddress"] = "127.0.0.1"
    optionsDict["use_multicast"] = True

    # This will create a new NatNet client
    optionsDict = my_parse_args(sys.argv, optionsDict)

    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])


    streaming_client.rigid_body_listener = experiment.process_optitrack_rigid_body_data
    streaming_client.lacrosse_listener = experiment.process_optitrack_ball_data
    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()


    command_thread = Thread(target=experiment.handle_command)
    command_thread.start()
    move_robot_thread = Thread(target=experiment.move_robot)
    move_robot_thread.start()
    move_arm_thread = Thread(target=experiment.move_robot)
    move_arm_thread.start()



