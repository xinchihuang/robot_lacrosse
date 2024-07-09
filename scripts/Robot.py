import math
import sys
import time
import numpy as np
from squaternion import Quaternion
from scripts.optitrack_sdk.NatNetClient import NatNetClient
from scripts.optitrack_control.robomaster_executor import RoboMasterExecutor
from scripts.data_process.check_parabola_point import check_parabola_point
import os
from scripts.manual_control.throw import throw_a_ball

# This is a callback function that gets connected to the NatNet client
# and called once per mocap frame.

def optitrack_coordinate_to_world_coordinates(position, rotation):
    q = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])
    euler_angles = q.to_euler(degrees=False)
    x_world=position[0]
    y_world=-position[2]
    z_world=position[1]
    theta_world=euler_angles[1]
    return x_world, y_world, z_world,-theta_world
def global_control_to_local_control(self_rotation,controls):
    vx_local=controls[0]*math.cos(self_rotation)+controls[1]*math.sin(self_rotation)
    vy_local=-controls[0]*math.sin(self_rotation)+controls[1]*math.cos(self_rotation)
    omega_local=controls[2]
    return vx_local, vy_local, omega_local
def central_controller(self_position,self_rotation,target_position,target_rotation=0,max_speed=3.5,decrease_range=0.5):

    dx=target_position[0] - self_position[0]
    dy=target_position[1] - self_position[1]
    # print("max_speed", math.sqrt(dx ** 2 + dy ** 2))
    if math.sqrt(dx**2+dy**2) > decrease_range:

        vx = max_speed * dx / math.sqrt(dx ** 2 + dy ** 2)
        vy = max_speed * dy / math.sqrt(dx ** 2 + dy ** 2)
    else:
        vx = max_speed * dx
        vy = max_speed * dy

    omega = target_rotation - self_rotation
    vx_local,vy_local,omega_local = global_control_to_local_control(self_rotation,[vx,vy,omega])
    return vx_local, vy_local, omega_local

def landing_point_predictor_old(ball_memory,arm_hieght=0.3):
    g_x=0
    g_y=0
    g_z=-10
    arm_pose=[-0.2,0,0.3]
    t_start = ball_memory[0][3]
    A=[]
    B=[]
    landing_target_x, landing_target_y,drop_t=None,None,None
    for i in range(len(ball_memory)):
        t = ball_memory[i][3] - t_start
        A.append([1, t, 0, 0, 0, 0])
        A.append([0, 0, 1, t, 0, 0])
        A.append([0, 0, 0, 0, 1, t])
        B.append([ball_memory[i][0]-0.5*g_x*t*t])
        B.append([ball_memory[i][1]-0.5*g_y*t*t])
        B.append([ball_memory[i][2]-0.5*g_z*t*t])

    A_array = np.array(A)
    B_array = np.array(B)
    pseudo_inverse_A = np.linalg.pinv(A_array)
    solved_parameters = pseudo_inverse_A @ B_array
    x0, vx, y0, vy, z0, vz = solved_parameters[0][0], solved_parameters[1][0], solved_parameters[2][0], \
    solved_parameters[3][0], solved_parameters[4][0], solved_parameters[5][0]

    if z0 ** 2 - (z0 - arm_pose[2]) * (g_z) * 2 > 0:
        drop_t = (-vz - math.sqrt(vz ** 2 - (z0 - arm_pose[2]) * (g_z) * 2)) / (g_z)
        landing_target_x = x0 + drop_t * vx
        landing_target_y = y0 + drop_t * vy

    return landing_target_x, landing_target_y,drop_t
def root(a,b,c):
    if b**2 - 4*a*c<0:
        return None,None
    return (-b + math.sqrt(b**2 - 4*a*c))/2/a,(-b - math.sqrt(b**2 - 4*a*c))/2/a
def landing_point_predictor(ball_memory,arm_hieght=0.3):
    ball_memory=np.array(ball_memory)
    x = ball_memory[:, 0]
    y = ball_memory[:, 1]
    z = ball_memory[:, 2]

    # Fit a second-degree polynomial (parabola) to the y and z coordinates
    coefficients_xy = np.polyfit(x, y, 1)
    a, b = coefficients_xy  # Extract coefficients
    coefficients_xz = np.polyfit(x, z, 2)
    c, d, e = coefficients_xz  # Extract coefficients
    coefficients_yz = np.polyfit(y, z, 2)
    f, g, h = coefficients_yz  # Extract coefficients
    # Generate y values for the fit
    x1,x2=root(c,d,e-arm_hieght)
    y1,y2=root(f,g,h-arm_hieght)
    if x1==None or x2==None or y1==None or y2==None:
        # print("Error", len(ball_memory))
        return ball_memory[-1][0],ball_memory[-1][1],1
    x0=ball_memory[0][0]
    y0=ball_memory[0][1]
    d1 = (x1 - x0) ** 2 + (y1 - y0) ** 2
    d2 = (x2 - x0) ** 2 + (y1 - y0) ** 2
    d3 = (x1 - x0) ** 2 + (y2 - y0) ** 2
    d4 = (x2 - x0) ** 2 + (y2 - y0) ** 2
    if max(d1,d2,d3,d4)==d1:
        landing_x=x1
        landing_y=y1
    elif max(d1,d2,d3,d4)==d2:
        landing_x=x2
        landing_y=y1
    elif max(d1,d2,d3,d4)==d3:
        landing_x=x1
        landing_y=y2
    elif max(d1,d2,d3,d4)==d4:
        landing_x=x2
        landing_y=y2
    # print(x1,x2,y1,y2)
    return landing_x,landing_y,1




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

    def process_optitrack_rigid_body_data(self, new_id, position, rotation):
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



