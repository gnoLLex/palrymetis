import time
import torch
import spdlog
import signal
import threading

import torchcontrol as toco

from palrymetis.panda import Panda
from palrymetis.controllers import HumanController, ImitationController
from polymetis import RobotInterface

import math

def generate_sinusoidal_value(frequency=0.0005, phase=0, t=0):
    # Generate a sinusoidal value in the range of 0.015 to 0.075
    amplitude = (0.075 - 0.015) / 2
    offset = 0.015 + amplitude  # Center of the range

    sinusoidal_value = amplitude * math.sin(2 * math.pi * frequency * t + phase) + offset
    return sinusoidal_value

GRIPPER_HZ = 3
GRIPPER_LOWER_BOUND = 0.015
GRIPPER_UPPER_BOUND = 0.075
GRIPPER_SPEED_MIN = 0.0
GRIPPER_SPEED_MAX = 0.07
GRIPPER_FORCE_MIN = 30.0
GRIPPER_FORCE_MAX = 70.0
GRIPPER_GRASP_THRESHOLD = 0.01
GRIPPER_OPEN_THRESHOLD = 0.05
NEXT_GRIPPER_COMMAND_WIDTH_THRESHOLD = 0.0005

class Teleoperation:
    def __init__(self, operated_panda: Panda, imitator_panda: Panda):
        self.logger = spdlog.ConsoleLogger("teleoperation")
        self.logger.set_level(spdlog.LogLevel.DEBUG)
        self.operated_panda = operated_panda
        self.imitator_panda = imitator_panda

        self.human_controller = HumanController(self.operated_panda.robot)
        self.imitation_controller = ImitationController(self.imitator_panda.robot)

        # Reset imitator to operated robots current positions
        self.logger.info(f"Moving {self.imitator_panda.name} to {self.operated_panda.name}'s joint positions")
        self.imitator_panda.robot.move_to_joint_positions(self.operated_panda.robot.get_joint_positions())

        self.logger.info("Sending torch policies")
        self.operated_panda.load_policy(self.human_controller, blocking=False)
        self.imitator_panda.load_policy(self.imitation_controller, blocking=False)

        self.running = True

        # setup up signal for cleanup
        signal.signal(signal.SIGINT, self.__sig_handler)

    def run(self):
        self.running = True
        last_issued = -1
        timestamp = time.time()
        t = 0
        while self.running:
            if not self.operated_panda.is_running_policy():
                self.operated_panda.load_policy()

            if not self.imitator_panda.is_running_policy():
                self.imitator_panda.load_policy()

            # update robot target position
            joint_pos_desired = self.operated_panda.robot.get_joint_positions()
            self.imitator_panda.robot.update_desired_joint_positions(joint_pos_desired)

            # mirror gripper state for now only binary (grasping fully or moving to fully open)
            # desired_gripper_width = self.operated_panda.get_state()["gripper"].width
            desired_gripper_width = generate_sinusoidal_value(t=t)

            
            now = time.time()
            if abs(last_issued - desired_gripper_width) > NEXT_GRIPPER_COMMAND_WIDTH_THRESHOLD and now - timestamp >= 1 / GRIPPER_HZ:
                print(desired_gripper_width)
                self.imitator_panda.gripper.goto(
                    width=min(GRIPPER_UPPER_BOUND, desired_gripper_width),
                    speed=GRIPPER_SPEED_MAX,
                    force=GRIPPER_FORCE_MAX,
                )
                timestamp = now
            t += 1
            

    def __sig_handler(self, _sig, _frame):
        self.running = False

    def cleanup(self):
        self.logger.info("Terminating policies")
        self.operated_panda.robot.terminate_current_policy()
        self.imitator_panda.robot.terminate_current_policy()

