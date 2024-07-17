import json
class ControlData:
    """
    A data structure for passing control signals to executor
    """

    def __init__(self):
        self.robot_index = None
        # self.omega_left = 0
        # self.omega_right = 0

        self.velocity_x = 0
        self.velocity_y = 0
        self.omega=0

class ControlDataXYOmega:
    """
    A data structure for passing control signals to executor
    """

    def __init__(self):
        self.robot_index = None
        # self.omega_left = 0
        # self.omega_right = 0

        self.velocity_x = 0
        self.velocity_y = 0
        self.omega=0

class SensorData:
    """
    A class for record sensor data
    """

    def __init__(self):
        self.robot_index = None
        self.position = None
        self.orientation = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.occupancy_map = None


class SceneData:
    """
    A class for passing data from scene
    """

    def __init__(self):
        self.observation_list = None
        self.adjacency_list = None
        self.position_list = None
        self.orientation_list = None
class Command:
    def __init__(self,state="idle",vx=0,vy=0,vw=0,r1=0,r2=-90,r3=180,target_x=0,target_y=0,rv1=0,rv2=0,rv3=0):

        self.state = state
        self.vx = vx
        self.vy = vy
        self.vw = vw
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3
        self.rv1 = rv1
        self.rv2 = rv2
        self.rv3 = rv3
        self.target_x = target_x
        self.target_y = target_y

    def encode(self):

        return json.dumps(self.__dict__)
    def set_attribute(self, **kwargs):
        self.state = kwargs.get('state')
        self.vx = float(kwargs.get('vx'))
        self.vy = float(kwargs.get('vy'))
        self.vw = float(kwargs.get('vw'))
        self.r1 = float(kwargs.get('r1'))
        self.r2 = float(kwargs.get('r2'))
        self.r3 = float(kwargs.get('r3'))
        self.rv1 = float(kwargs.get('rv1'))
        self.rv2 = float(kwargs.get('rv2'))
        self.rv3 = float(kwargs.get('rv3'))
        self.target_x = float(kwargs.get('target_x'))
        self.target_y = float(kwargs.get('target_y'))

    def decode(self,command_string):
        attribute_dict = json.loads(command_string)
        self.set_attribute(**attribute_dict)

