from polymetis import RobotInterface, GripperInterface
from polymetis_pb2 import RobotState
from typing import List, Dict

import torchcontrol as toco
import torch
import spdlog

# TODO: automatically restart controller if server complains
class Panda:
    def __init__(self, name, ip, robot_port, gripper_port):
        self.name = name
        self.logger = spdlog.ConsoleLogger(name)
        self.robot = RobotInterface(
            ip_address=ip,
            port=robot_port
        )
        self.gripper = GripperInterface(
            ip_address=ip,
            port=gripper_port
        )
        self.robot_policy = None

    @classmethod
    def from_alr_name(cls, name):
        import sys, yaml
        # TODO: config path (maybe env var?)
        panda_config = yaml.load(open("configs/robots.yaml"), yaml.FullLoader)[name]

        ip = "localhost" # TODO: localhost for now, ip of rt PC otherwise

        panda_robot_port = panda_config["robot_port"]
        panda_gripper_port = panda_config["gripper_port"]

        return cls(name, ip, panda_robot_port, panda_gripper_port)

