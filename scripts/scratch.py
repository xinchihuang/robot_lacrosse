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
print(np.load("saved_robot_data/0.npy"))
