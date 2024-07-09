import math
import numpy as np
from squaternion import Quaternion


def optitrack_coordinate_to_world_coordinates(position, rotation):
    """
    Converts a rigid body position or point position from optitrack coordinate to world coordinates
    Args:
        position: [x,y,z]
        rotation:  Quaternion (w,x,y,z)

    Returns:

    """
    q = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])
    euler_angles = q.to_euler(degrees=False)
    x_world=position[0]
    y_world=-position[2]
    z_world=position[1]
    theta_world=euler_angles[1]
    return x_world, y_world, z_world,-theta_world
def global_control_to_local_control(self_rotation,controls):
    """
    Transfer the global control to robot's local control
    Args:
        self_rotation: robots rotation relative to z axis (in radius)
        controls: global control

    Returns:

    """
    vx_local=controls[0]*math.cos(self_rotation)+controls[1]*math.sin(self_rotation)
    vy_local=-controls[0]*math.sin(self_rotation)+controls[1]*math.cos(self_rotation)
    omega_local=controls[2]
    return vx_local, vy_local, omega_local
def distance_controller_local(self_position,target,control_gain=1,max_speed=1.5):
    """
    A simple distance based controller
    Args:
        self_position: robot position in robots coordinate frame [x,y,(z),(t)]
        target: target position in robots coordinate frame [x,y,(z),(t)]
        control_gain: A parameter for controlling how fast the velocity change according to relative distance
        max_speed: The maximum speed of robot

    Returns:
        velocities: [vx,vy,...]
    """
    distance_x = target[0] - self_position[0]
    distance_y = target[1] - self_position[1]
    vx = min(abs(distance_x) * control_gain, 1) * max_speed * (distance_x) / abs(distance_x)
    vy = min(abs(distance_y) * control_gain, 1) * max_speed * (distance_y) / abs(distance_y)
    return [vx,vy]
def distance_time_controller_local(self_position,target,t,max_speed=1.5):
    """
    A simple distance based controller with time
    Args:
        self_position: robot position in robots coordinate frame [x,y,(z),(t)]
        target: target position in robots coordinate frame [x,y,(z),(t)]
        t: The time to reach the target
        max_speed: The maximum speed of robot

    Returns:
        velocities: [vx,vy,...]
    """
    distance_x = target[0] - self_position[0]
    distance_y = target[1] - self_position[1]
    vx = (distance_x/t)/abs(distance_x/t)*max(abs(distance_x/t),max_speed)
    vy = (distance_y/t)/abs(distance_y/t)*max(abs(distance_y/t),max_speed)
    return [vx,vy]
def central_controller(self_position,self_rotation,target_position,target_rotation=0,max_speed=3.5,decrease_range=0.5):
    """
    Central controller function for chassis, try to maximize the speed before reach the decrease range
    Args:
        self_position: robot present position in world coordinates
        self_rotation:  robot present rotation in world coordinates
        target_position: target position in world coordinates
        target_rotation: target rotation in world coordinates
        max_speed: the maximum speed of the chassis
        decrease_range: decrease range of the chassis

    Returns:

    """
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
    """
    Calculates the roots of the parabola equation
    If no real roots are found return None
    Args:
        a:
        b:
        c:

    Returns: roots

    """
    if b**2 - 4*a*c<0:
        return None,None
    return (-b + math.sqrt(b**2 - 4*a*c))/2/a,(-b - math.sqrt(b**2 - 4*a*c))/2/a
def landing_point_predictor(ball_memory,arm_hieght=0.3):
    """
    Calculates the landing point of the ball
    (need to add time predictor later)
    Args:
        ball_memory: the observed ball memory
        arm_hieght: the net's height

    Returns:

    """

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

