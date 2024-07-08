

# move conrtollers
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

# arm controllers
def arm_torque_controller(motor_list,desired_angle,desired_speed):
    pass

# landing prediction
# def centralized_parabolic_prediction(ball_position,ball_velocity):
#     if vz_ball_w ** 2 - (z_ball_w - self.arm_pose[2]) * (-self.g) * 2 > 0:
#         drop_t = (-vz_ball_w - math.sqrt(
#             vz_ball_w ** 2 - (z_ball_w - self.arm_pose[2]) * (-self.g) * 2)) / (-self.g)
#         target_x = x_ball_w + drop_t * vx_ball_w - self.arm_pose[0]
#         target_y = y_ball_w + drop_t * vy_ball_w - self.arm_pose[1]
#     self.move_target = [target_x, target_y]
