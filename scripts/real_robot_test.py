import math
import sys
from scripts.optitrack_sdk.NatNetClient import NatNetClient
from scripts.robomaster_executor.robomaster_executor import RoboMasterExecutor
from scripts.arm_control.throw import throw_a_ball
from Robot import Robot
from scripts.arm_control.arm_executor import ArmExecutor
from scripts.chassis_control.chassis_controller import optitrack_coordinate_to_world_coordinates
from scripts.data_process.check_parabola_point import check_parabola_point
from threading import Thread
from utils import calculate_rotation_angle
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
def cal_angle_speed(h,d,g=9.8):
    v_vertical=math.sqrt(2*g*h)
    t=math.sqrt(8*h/g)
    v_horizon=d/t
    speed=math.sqrt(v_horizon**2+v_vertical**2)
    angle=math.atan(v_vertical/v_horizon)
    return angle,speed
class Experiment:
    def __init__(self):
        self.ball_memory=[]
        self.robot_list=[]
        self.throwing=False
        self.check_point_window_size=30
        self.check_parabola_window_size = 10

        ##save_related
        self.saved=False
        self.saved_ball_data=[]
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
                try:
                    thrower_name=command_list[1]

                    if len(self.robot_list) == 2:
                        robot1 = self.robot_list[0]
                        robot2 = self.robot_list[1]
                        if not robot1.robot_self_pose is None and not robot2.robot_self_pose is None and robot1.state == "rotate" and robot2.state == "rotate":
                            # print(robot1.robot_name, robot1.robot_self_pose)
                            # print( robot2.robot_name, robot2.robot_self_pose)
                            if "1" == str(thrower_name):
                                distance=math.sqrt((robot1.robot_self_pose[0]-robot2.robot_self_pose[0])**2+(robot1.robot_self_pose[1]-robot2.robot_self_pose[1])**2)
                                desired_angle,desired_speed=cal_angle_speed(1.5,distance)
                                arm_msg=robot1.arm_throw_ball(desired_angle,desired_speed)
                                print(arm_msg)
                                self.saved_arm_input=[1,desired_angle,desired_speed]
                                robot2.state="catch"
                            elif "2" == str(thrower_name):
                                distance = math.sqrt((robot1.robot_self_pose[0] - robot2.robot_self_pose[0]) ** 2 + (
                                            robot1.robot_self_pose[1] - robot2.robot_self_pose[1]) ** 2)
                                desired_angle, desired_speed = cal_angle_speed(1.5, distance)
                                arm_msg=robot2.arm_throw_ball(desired_angle,desired_speed)
                                print(arm_msg)
                                self.saved_arm_input=[2, desired_angle, desired_speed]
                                robot1.state = "catch"
                    self.throwing=True
                except:
                    print("Invalid command")
                    continue
            elif state == "reset":
                for robot in self.robot_list:
                    robot.state = state
                    robot.reset_arm()
            elif state == "rotate":
                for robot in self.robot_list:
                    robot.state = state

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

                robot.robot_self_pose=[x_world, y_world, z_world, theta_world]

                if robot.state=="catch":
                    robot.ball_memory=self.ball_memory[6:self.check_point_window_size+6]

                    vx, vy, omega=robot.generate_cotrol(x_world, y_world, z_world, theta_world)
                    # print(vx,vy,omega)
                    robot.execute(vx, vy, omega)
        # robot1 = self.robot_list[0]
        # robot2 = self.robot_list[1]
        # print(robot1.robot_self_pose, robot2.robot_self_pose)
        if len(self.robot_list) == 2:
            robot1 = self.robot_list[0]
            robot2 = self.robot_list[1]
            if not robot1.robot_self_pose is None and not robot2.robot_self_pose is None and robot1.state== "rotate" and  robot2.state== "rotate":
                # print(robot1.robot_name, robot1.robot_self_pose)
                # print( robot2.robot_name, robot2.robot_self_pose)
                if "1" == str(id):
                    vx, vy, omega=robot1.rotate(robot1.robot_self_pose, robot2.robot_self_pose)
                    robot1.execute(vx, vy, omega)
                elif "2" == str(id):
                    vx, vy, omega=robot2.rotate(robot2.robot_self_pose, robot1.robot_self_pose)
                    robot2.execute(vx, vy, omega)

    def process_optitrack_ball_data(self, mocap_data):
        """

        Args:
            mocap_data: process ball data

        Returns:

        """
        position = None
        rotation = None
        z_world = None
        for marker_id in range(len(mocap_data.labeled_marker_data.labeled_marker_list)):
            if mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1] > 0.25:
                position = [mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[0],
                            mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1],
                            mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[2]]
                rotation = [0, 0, 0, 0]

        # print(position)
        if not position == None and self.throwing==True:
            x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation)
            if z_world > 0.35 and x_world ** 2 + y_world ** 2 < 4:
                present_time = time.time()
                self.ball_memory.append([x_world, y_world, z_world, present_time])
                is_parabola = check_parabola_point(self.ball_memory)
                # print(is_parabola,len(self.ball_memory))
                if not is_parabola and len(self.ball_memory) > self.check_parabola_window_size:
                    self.ball_memory = []
                # print(z_world, len(self.ball_memory))
                if self.saved == False:
                    self.saved_ball_data.append([x_world, y_world, z_world])


            else:
                if len(self.saved_ball_data) > 50 and self.saved == False:
                    files_and_dirs = os.listdir(
                        "saved_ball_data/")
                    # Filter out directories, keeping only files
                    files = [f for f in files_and_dirs if os.path.isfile(
                        os.path.join("saved_ball_data/", f))]
                    number = len(files)
                    np.save("./saved_ball_data/" + str(
                        number) + ".npy", np.array(robot1.save_data))
                    np.save("./saved_arm_data/" + str(
                        number) + ".npy", np.array(self.saved_arm_input))
                    print("saved " + str(number))
                    self.saved = True
                self.ball_memory = []
if __name__ == "__main__":
    robot1_chassis_executor=RoboMasterExecutor(sn="3JKCH8800101C2")
    # robot1_arm_executor = ArmExecutor(('192.168.0.105', 12345))
    # robot1_chassis_executor=None
    robot1_arm_executor = None
    # robot1_executor=None
    robot1 = Robot('1',robot1_chassis_executor,robot1_arm_executor)

    robot2_chassis_executor = RoboMasterExecutor(sn="3JKCH7T00100M9")
    # robot2_arm_executor = ArmExecutor(('192.168.0.104', 12345))
    # robot2_chassis_executor = None
    robot2_arm_executor=None
    robot2 = Robot('2', robot2_chassis_executor,robot2_arm_executor)
    experiment=Experiment()
    experiment.robot_list.append(robot1)
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



