import sys
from scripts.optitrack_sdk.NatNetClient import NatNetClient
from scripts.robomaster_executor.robomaster_executor import RoboMasterExecutor
from scripts.arm_control.throw import throw_a_ball
from Robot import Robot
from arm_control.arm_manager import Arm
import os
import numpy as np
import random
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
    # robot1_executor=RoboMasterExecutor()
    robot1_executor=None
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


    # streaming_client.rigid_body_listener = robot1.process_optitrack_rigid_body_data
    streaming_client.lacrosse_listener = robot1.process_optitrack_ball_data
    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run()

    arm = Arm()
    step=250
    angle_1=0
    angle_2=90
    angle_3=0
    brake_angle = random.randint(55,65)
    # brake_angle=65
    torque = random.uniform(-12,-10)
    arm.go_to([[step, angle_1, angle_2, angle_3]])
    arm_data = arm.throw_to_angle_with_torque(brake_angle=brake_angle, torque=torque)
    arm.go_to([[step, angle_1, angle_2, angle_3]])
    arm_data.append([angle_1,0,0,2])
    arm_data.append([angle_2,0,0,3])
    arm_data.append([angle_3,0,0,4])
    arm_data.append([brake_angle,0,0,5])
    arm_data.append([torque,0,0,6])
    print(arm_data)
    print(robot1.save_data)

    files_and_dirs = os.listdir(
        "saved_ball_data/")
    # Filter out directories, keeping only files
    files = [f for f in files_and_dirs if os.path.isfile(
        os.path.join("saved_ball_data/", f))]
    number = len(files)
    np.save("./saved_ball_data/" + str(
        number) + ".npy", np.array(robot1.save_data))
    np.save("./saved_arm_data/" + str(
        number) + ".npy", np.array(arm_data))
    print("saved " + str(number))
