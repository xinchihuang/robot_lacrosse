import sys
from scripts.optitrack_sdk.NatNetClient import NatNetClient
from scripts.robomaster_executor.robomaster_executor import RoboMasterExecutor
from scripts.arm_control.throw import throw_a_ball
from Robot import Robot
from scripts.arm_control.arm_executor import ArmExecutor
from scripts.chassis_control.chassis_controller import optitrack_coordinate_to_world_coordinates
from scripts.data_process.check_parabola_point import check_parabola_point
from threading import Thread
import time
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
class Experiment:
    def __init__(self):
        self.ball_memory=[]
        self.robot_list=[]
    def handle_command(self):
        """
        Handles commands name,state(catch,reset,throw),extra command(desired arm angle,desired speed)
        Returns:

        """
        while True:
            command=input("Press enter command: ")
            command_list=command.split(",")
            robot_name=command_list[0]
            for robot in self.robot_list:
                if robot.robot_name==robot_name:
                    state=command_list[1]
                    if state == "catch":
                        robot.state=state
                    elif state=="reset":
                        robot.state = state
                        robot.reset_arm()
                    elif state=="throw":
                        robot.state = state
                        angle=float(command_list[2])
                        speed=float(command_list[3])
                        robot.arm_throw_ball(angle,speed)
                print(command_list)
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
                if robot.state=="catch":
                    robot.ball_memory=self.ball_memory
                    vx, vy, omega=robot.generate_cotrol(x_world, y_world, z_world, theta_world)
                    print(vx,vy,omega)
                    robot.execute(vx, vy, omega)

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
        if not position == None:
            x_world, y_world, z_world, theta_world = optitrack_coordinate_to_world_coordinates(position, rotation)
            if z_world > 0.35 and x_world ** 2 + y_world ** 2 < 4:
                present_time = time.time()
                self.ball_memory.append([x_world, y_world, z_world, present_time])
                is_parabola = check_parabola_point(self.ball_memory)
                # print(is_parabola,len(self.ball_memory))
                if not is_parabola and len(self.ball_memory) > 20:
                    self.ball_memory = []
                # print(z_world, len(self.ball_memory))
                # if self.saved == False:
                #     self.save_data.append([x_world, y_world, z_world])


            else:
                # if len(self.save_data) > 50 and self.saved == False:
                #     self.saved = True
                self.ball_memory = []
if __name__ == "__main__":
    # robot1_chassis_executor=RoboMasterExecutor()
    robot1_arm_executor = ArmExecutor(('192.168.0.105', 12345))
    robot1_chassis_executor=None
    # robot1_arm_executor = None
    # robot1_executor=None
    robot1 = Robot('1',robot1_chassis_executor,robot1_arm_executor)

    # robot2_chassis_executor = RoboMasterExecutor()
    # robot2_arm_executor = ArmExecutor(('192.168.0.104', 12345))
    # robot2_arm_executor = None
    # robot1_executor=None
    # robot2 = Robot('2', robot2_chassis_executor,robot2_arm_executor)
    experiment=Experiment()
    experiment.robot_list.append(robot1)
    # experiment.robot_list.append(robot2)

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



