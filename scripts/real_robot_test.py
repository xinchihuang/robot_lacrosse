import math
import random
import sys
from scripts.optitrack_sdk.NatNetClient import NatNetClient
from scripts.robomaster_executor.robomaster_executor import RoboMasterExecutor
from Robot import *
from scripts.arm_control.arm_executor import *
from scripts.utils import optitrack_coordinate_to_world_coordinates
from scripts.data_process.check_parabola_point import check_parabola_point
from threading import Thread
import os
import time
import numpy as np
from utils import *
from throw_ml import *
import signal
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
    ball_memory=np.array(ball_memory)
    if len(ball_memory)<check_window:
        return False
    else:
        sma=simple_moving_average(ball_memory[:,2])
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
        self.model=SimpleMLP(input_dim=2, hidden_dim=20, output_dim=2)
        self.model.load_state_dict(torch.load("save_model_throw.pth"))

        ##save_related
        self.saved_ball_data = []
        self.saved_robot_data = []
        self.saved_arm_input=[]
        self.saved_arm_record=[]

        self.is_running=True

    def handle_command(self):
        """
        Handles commands name,state(catch,reset,throw),extra command(desired arm angle,desired speed)
        Returns:

        """
        while self.is_running:
            command=input("Press enter command: ")
            command_list=command.split(",")
            state=command_list[0]
            if state == "1":
                command_list=["throw","2","1"]
            elif state == "2":
                command_list=["throw","1","2"]
            ### throw case
            if state == "throw" or state=="1" or state=="2":
                if len(self.robot_list) == 1:
                    robot=self.robot_list[0]
                    robot.robot_state = "throw"
                elif len(self.robot_list) == 2:
                    thrower_name = command_list[1]
                    catcher_name = command_list[2]
                    for robot in self.robot_list:
                        if thrower_name==robot.name:
                            robot.robot_state = "throw"
                        if catcher_name==robot.name:
                            robot.robot_state ="catch"
                self.state="throw"
                if len(self.robot_list) == 1:
                    robot=self.robot_list[0]
                    if robot.robot_state=="throw":
                        self.throw_h, distance=1.5,2
                        desired_angle, desired_speed = cal_angle_speed(self.throw_h, distance)

                        desired_angle=35
                        desired_speed=30
                        robot.arm_throw_ball(desired_angle, desired_speed)
                        # self.arm_msg=arm_msg.decode()
                        # print(self.arm_msg)
                        self.saved_arm_input = [1, desired_angle, desired_speed]

                else:
                    thrower=None
                    catcher=None
                    for robot in self.robot_list:
                        if robot.robot_state=="throw":
                            thrower=robot
                        elif robot.robot_state=="catch":
                            catcher=robot
                    if not thrower is None and not catcher is None:
                        # print(thrower.name, catcher.name)
                        distance = math.sqrt((thrower.robot_self_pose[0] - catcher.robot_self_pose[0]) ** 2 + (
                                    thrower.robot_self_pose[1] - catcher.robot_self_pose[1]) ** 2)
                        desired_angle, desired_speed = cal_angle_speed(self.throw_h, distance)
                        feature = [[self.throw_h, distance]]
                        feature = torch.tensor(feature)
                        output = self.model(feature)
                        min_angle, max_angle, min_vel, max_vel = 25, 35, 29, 31
                        residual = linear_mapping(output.squeeze().detach().numpy()[0], 0, 1.0, min_angle,
                                                         max_angle)
                        desired_angle = desired_angle + residual
                        desired_speed = 30
                        thrower.arm_throw_ball(desired_angle, desired_speed)
                        # print(desired_angle, desired_speed, distance)
                        # desired_speed = desired_speed
                        self.saved_arm_input = [1, desired_angle, desired_speed]
                self.throw_starts_time=time.time()
            ### reset case
            elif state == "reset":
                self.state="reset"
                for robot in self.robot_list:
                    robot.robot_state = state
                    robot.reset_arm()

                if self.state == "reset":
                    self.throwing = False
                    for robot in self.robot_list:
                        robot.robot_state = "reset"
                        robot.reset_arm()
                    self.state = "idle"
            ### launch case
            # elif state == "launch":
            #     if len(self.robot_list) == 1:
            #         robot=self.robot_list[0]
            #         robot.robot_state = "launch"
                # elif len(self.robot_list) == 2:
                #     thrower_name = command_list[1]
                #     catcher_name = command_list[2]
                #     for robot in self.robot_list:
                #         if thrower_name==robot.name:
                #             robot.robot_state = "launch"
                #         if catcher_name==robot.name:
                #             robot.robot_state ="catch"
                # # print(self.state)
                # thrower = None
                # for robot in self.robot_list:
                #     # print(robot.robot_state)
                #     # print("AAAA")
                #     if robot.robot_state == "launch":
                #         thrower = robot
                # if not thrower is None:
                #     self.throw_h, distance = 1.5, 5
                #     desired_angle, desired_speed = cal_angle_speed(self.throw_h, distance, arm_length=0.3)
                #
                #     thrower.launcher_throw_ball(desired_angle, desired_speed)
                #     self.saved_arm_input = [1, desired_angle, desired_speed]
                # self.throw_starts_time = time.time()

            elif state == "catch":
                # if len(self.robot_list) == 1:
                robot=self.robot_list[0]
                robot.robot_state = "catch"
                self.state="catch"
                self.throw_starts_time=time.time()
            elif state == "rotate":
                self.state="rotate"
                for robot in self.robot_list:
                    robot.robot_state = "rotate"


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
            if robot.name == str(id):
                x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation)
                robot.robot_self_pose = [x_world, y_world, z_world, theta_world]
                # if self.state == "throw" or self.state =="catch" or self.state=="launch":
                #     self.saved_robot_data.append([id, x_world, y_world, z_world])
        if len(self.robot_list) == 2 and self.state == "rotate":
            # print(self.state)

            robot1 = self.robot_list[0]
            robot2 = self.robot_list[1]
            # print(robot1.robot_state,robot2.robot_state)
            if not robot1.robot_self_pose is None and not robot2.robot_self_pose is None:

                vx, vy, omega = robot1.get_rotate_control(robot2.robot_self_pose)
                # print(vx, vy, omega)
                robot1.execute(vx, vy, omega)
                vx, vy, omega = robot2.get_rotate_control(robot1.robot_self_pose)
                robot2.execute(vx, vy, omega)
        elif self.state == "throw":
            for robot in self.robot_list:
                if robot.robot_state == "catch":
                    vx, vy, omega = robot.get_move_control(self.ball_memory[:self.check_point_window_size])
                    save_robot_data = [float(robot.name), robot.robot_self_pose[0], robot.robot_self_pose[1],
                                       robot.robot_self_pose[2], robot.robot_self_pose[3]]
                    self.saved_robot_data.append(save_robot_data)
                    # print(robot.name,vx,vy,omega)
                    robot.execute(vx, vy, omega)
        elif self.state == "catch":
            # print(self.state)
            for robot in self.robot_list:
                # print(robot.robot_state)
                if robot.robot_state == "catch":
                    vx, vy, omega = robot.get_move_control(self.ball_memory[:self.check_point_window_size])
                    save_robot_data=[float(robot.name),robot.robot_self_pose[0],robot.robot_self_pose[1],robot.robot_self_pose[2],robot.robot_self_pose[3]]
                    self.saved_robot_data.append(save_robot_data)
                    # print(robot.robot_self_pose,vx,vy,omega)
                    # print(vx,vy,omega)
                    # print(time.time()-self.throw_starts_time)
                    robot.execute(vx, vy, omega)
        elif self.state == "launch":
            pass
        elif self.state == "idle":

            for robot in self.robot_list:
                robot.robot_state = "idle"
                robot.execute(0, 0, 0)

    def process_optitrack_ball_data(self, mocap_data):
        """

        Args:
            mocap_data: process ball data

        Returns: None

        """
        if self.state == "rotate":
            pass

        elif time.time()-self.throw_starts_time<3:
            position = None
            rotation = None
            if self.state=="throw" or self.state=="launch" or self.state=="catch":
                # print(self.state)
                ### filter out point that too low
                # print(len(mocap_data.labeled_marker_data.labeled_marker_list))
                for marker_id in range(len(mocap_data.labeled_marker_data.labeled_marker_list)):

                    if mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1] > 0.4:
                        # print(len(mocap_data.labeled_marker_data.labeled_marker_list))
                        position = [mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[0],
                                    mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1],
                                    mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[2]]
                        rotation = [0, 0, 0, 0]

                if not position == None:
                    x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation,is_ball=True)
                    # print(len(self.ball_memory))
                    self.ball_memory.append([x_world, y_world, z_world])
                else:

                    stop=check_ball_memory_records(self.ball_memory)
                    # print(position,stop)
                    if stop==True:
                        print("stop")
                        self.saved_ball_data = self.ball_memory
                        self.save_data()
                        self.ball_memory = []
                        self.saved_robot_data = []
                        self.state = "idle"
                    else:
                        pass
            else:
                # print("stop")
                self.saved_ball_data = self.ball_memory
                if len(self.saved_ball_data) > 30:
                    self.save_data()
                # self.state = "idle"
                self.ball_memory = []
                self.saved_robot_data = []
        else:
            # print("stop")
            self.saved_ball_data=self.ball_memory
            if len(self.saved_ball_data)>30:
                self.save_data()
            # self.state = "idle"
            self.ball_memory = []
            self.saved_robot_data = []



    def save_data(self):
        files_and_dirs = os.listdir(
            "./saved_ball_data/")
        # Filter out directories, keeping only files
        files = [f for f in files_and_dirs if os.path.isfile(
            os.path.join("./saved_ball_data/", f))]
        number = len(files)
        np.save("./saved_ball_data/" + str(
            number) + ".npy", np.array(self.saved_ball_data))
        np.save("./saved_robot_data/" + str(
            number) + ".npy", np.array(self.saved_robot_data))
        np.save("./saved_arm_data/" + str(
            number) + ".npy", np.array(self.saved_arm_input))

        print("saved " + str(number))
        self.saved_ball_data = []
        self.saved_robot_data = []
    def stop(self):
        self.running = False
        for robot in self.robot_list:
            robot.stop()
if __name__ == "__main__":
    # ball_launcher_chassis_executor = None
    # ball_launcher_arm_executor = LauncherExecutor(('192.168.0.106', 12345))
    robot1_chassis_executor=RoboMasterExecutor(sn="3JKCH8800101C2")
    robot1_arm_executor = ArmExecutor(('192.168.0.105', 12345))
    robot2_chassis_executor = RoboMasterExecutor(sn="3JKCH7T00100M9")
    robot2_arm_executor = ArmExecutor(('192.168.0.104', 12345))

    # robot1_chassis_executor=None
    # robot1_arm_executor = None
    # robot2_chassis_executor = None
    # robot2_arm_executor=None


    # launcher = Launcher('0', ball_launcher_chassis_executor, ball_launcher_arm_executor)
    robot1 = Robot('1', robot1_chassis_executor, robot1_arm_executor)
    robot2 = Robot('2', robot2_chassis_executor, robot2_arm_executor)
    experiment=Experiment()
    # experiment.robot_list.append(launcher)
    experiment.robot_list.append(robot1)
    experiment.robot_list.append(robot2)
    #
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
    # move_robot_thread = Thread(target=experiment.move_robot)
    # move_robot_thread.start()

    def make_handler(experiment,command_thread,streaming_client):
        def signal_handler(signum, frame):
            experiment.stop()
            command_thread.join()
            streaming_client.shutdown()
            print("Closing")
            sys.exit(0)

        return signal_handler


    stop_handler = make_handler(experiment,command_thread,streaming_client)
    signal.signal(signal.SIGINT, stop_handler)


