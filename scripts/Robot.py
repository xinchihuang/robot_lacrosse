import math
import sys
import time
import numpy as np
import os
from squaternion import Quaternion
from scripts.optitrack_sdk.NatNetClient import NatNetClient
from scripts.robomaster_executor import RoboMasterExecutor
from scripts.data_process.check_parabola_point import check_parabola_point
from scripts.manual_control.throw import throw_a_ball
from scripts.chassis_control.chassis_controller import optitrack_coordinate_to_world_coordinates,global_control_to_local_control,central_controller,landing_point_predictor

class Robot:
    def __init__(self, robot_name,executor=None):
        self.robot_name=robot_name
        self.executor=executor

        # General Settings
        self.g = 10
        self.max_speed = 3
        self.arm_pose = [-0.25, 0, 0.3]
        # landing prediction
        self.ball_memory=[]
        self.save_data=[]
        self.saved=False
        self.state=None
        self.parabola_state=False

    def process_optitrack_rigid_body_data(self, new_id, position, rotation):
        """

        Args:
            new_id: rigid body id
            position: rigid body position
            rotation: rigid body rotation

        Returns:

        """
        # print(position)
        x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation)
        if new_id==1:
            landing_target_x = None
            landing_target_y = None
            if len(self.ball_memory) >= 20:
                if check_parabola_point(self.ball_memory)==True:
                    landing_target_x, landing_target_y, drop_t = landing_point_predictor(self.ball_memory, self.arm_pose[2])
                # landing_target_x, landing_target_y, drop_t=0,0,1
                # landing_time = drop_t - (self.ball_memory[-1][3] - self.ball_memory[0][3])
            if x_world**2+y_world**2<2.25 and not landing_target_x==None and not landing_target_y==None:
                landing_target_x = landing_target_x - math.cos(theta_world) * self.arm_pose[0]
                landing_target_y = landing_target_y - math.sin(theta_world) * self.arm_pose[0]
                vx, vy, omega = central_controller([x_world, y_world, z_world], theta_world, [landing_target_x,landing_target_y, z_world], 0)
                # print("landing point",landing_target_x,landing_target_y)
                # if self.ball_memory[-1][2]<0.35:
                #     print("landing")
                # self.save_data.append(landing_time,landing_target_x,landing_target_y,self.ball_memory[-1][0],self.ball_memory[-1][1],self.ball_memory[-1][2])
                # print(landing_time,landing_target_x,landing_target_y,self.ball_memory[-1],len(self.ball_memory))
            else:
                vx, vy, omega=0,0,0
        else:
            vx, vy, omega = 0, 0, 0
        self.executor.execute([vx, vy, omega])
    #
    def process_optitrack_ball_data(self,mocap_data):
        """

        Args:
            mocap_data: process ball data

        Returns:

        """
        position=None
        rotation=None
        z_world=None
        for marker_id in range(len(mocap_data.labeled_marker_data.labeled_marker_list)):
            if mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1]>0.25:
                position=[mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[0],
                           mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[1],
                           mocap_data.labeled_marker_data.labeled_marker_list[marker_id].pos[2]]
                rotation=[0,0,0,0]

        # print(position)
        if not position==None:
            x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation)
            if z_world>0.33 and x_world ** 2 + y_world ** 2 < 2.25:
                present_time = time.time()
                self.ball_memory.append([x_world, y_world, z_world, present_time])
                is_parabola=check_parabola_point(self.ball_memory)
                print(is_parabola,len(self.ball_memory))
                if not is_parabola and len(self.ball_memory)>20:
                    self.ball_memory=[]
                # print(z_world,len(self.ball_memory))

                self.save_data.append([x_world, y_world, z_world])
                # if z_world < 0.35:
                #     print("landing")
                #     print(self.save_data[-1], len(self.ball_memory))


            else:
                if len(self.save_data)>50 and self.saved==False:
                    files_and_dirs = os.listdir("C:\\Users\\xinchi\\PycharmProjects\\optiitrack_global_control\\saved_data")
                    # Filter out directories, keeping only files
                    files = [f for f in files_and_dirs if os.path.isfile(os.path.join("C:\\Users\\xinchi\\PycharmProjects\\optiitrack_global_control\\saved_data", f))]
                    number=len(files)
                    np.save("C:\\Users\\xinchi\\PycharmProjects\\optiitrack_global_control\\saved_data\\"+str(number)+".npy", np.array(self.save_data))

                    self.saved=True
                self.ball_memory = []
                self.save_data=[]

            # self.executor.stop_robot()








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


if __name__ == "__main__":
    robot1_executor=RoboMasterExecutor()
    # robot1_executor=None
    robot1 = Robot('robot1',robot1_executor)





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


    streaming_client.rigid_body_listener = robot1.process_optitrack_rigid_body_data
    streaming_client.lacrosse_listener = robot1.process_optitrack_ball_data
    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    throw_a_ball()



