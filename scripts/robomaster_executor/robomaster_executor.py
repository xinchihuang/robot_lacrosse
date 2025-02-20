# -*-coding:utf-8-*-
# Copyright (c) 2020 DJI.
#
# Licensed under The 3-Clause BSD License (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     https://opensource.org/licenses/BSD-3-Clause
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import math
import time
from robomaster import robot
class  RoboMasterExecutor:
    def __init__(self,sn=None):
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="sta",sn=sn)
        self.ep_chassis = self.ep_robot.chassis
        self.max_speed=3.5
        self.max_rotation_speed=30
        self.sn=sn

    def execute(self,controls):

        vx=controls[0]
        vy=controls[1]
        vz=math.degrees(controls[2])
        if not vx==0:
            vx = min(abs(vx), self.max_speed) * abs(vx) / vx
        if not vy == 0:
            vy = min(abs(vy), self.max_speed) * abs(vy) / vy
        if not vz == 0:
            vz = min(abs(vz), self.max_rotation_speed) * abs(vz) / vz


        # self.ep_chassis.drive_speed(x=vx, y=-vy, z=-vz, timeout=0.02)
        # self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=5)
        # if not vx == 0 or not vy == 0 or not vz == 0:
        #     print(vx,vy,vz)
        self.ep_chassis.drive_speed(x=vx, y=-vy, z=-vz, timeout=1)
        # print(self.contols_count)
        # time.sleep(0.02)
    def stop(self):
        self.ep_chassis.drive_speed(x=0, y=0, z=0, timeout=1)
        self.ep_robot.close()