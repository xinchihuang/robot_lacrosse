import sys
from scripts.optitrack_sdk.NatNetClient import NatNetClient
from scripts.robomaster_executor.robomaster_executor import RoboMasterExecutor
from scripts.arm_control.throw import throw_a_ball
from Robot import Robot
from arm_control.arm_manager import Arm
import os
import numpy as np
import random
from scripts.real_robot_test import *
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
    # robot1_chassis_executor = RoboMasterExecutor(sn="3JKCH8800101C2")
    # robot1_arm_executor = ArmExecutor(('192.168.0.105', 12345))
    robot2_chassis_executor = None
    robot2_arm_executor = ArmExecutor(('192.168.0.104', 12345))
    # robot1_chassis_executor=None
    # robot1_arm_executor = None
    # robot2_chassis_executor = None
    # robot2_arm_executor=None

    # robot1 = Robot('1', robot1_chassis_executor, robot1_arm_executor)
    robot2 = Robot('2', robot2_chassis_executor, robot2_arm_executor)
    experiment = Experiment()
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
