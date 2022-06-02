import os
import numpy as np
from scipy.spatial.transform import Rotation as R

class Config:
    def __init__(self):
        self.package_name = "moveit_py3"

        # ros interface
        self.set_pose_srv_name = self.package_name+"/set_pose"
        self.get_pose_srv_name = self.package_name+"/get_pose"
        self.set_positions_srv_name = self.package_name+"/set_positions"
        self.get_positions_srv_name = self.package_name+"/get_positions"

config = Config()
