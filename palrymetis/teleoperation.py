import torch
import spdlog
import signal

import torchcontrol as toco

from typing import Dict

from palrymetis.panda import Panda
from polymetis import RobotInterface

class HumanController(toco.PolicyModule):
    # stolen from SimulationFramework
    def __init__(self, robot: RobotInterface, regularize=True):
        super().__init__()

        # get joint limits for regularization
        limits = robot.robot_model.get_joint_angle_limits()
        self.joint_pos_min = limits[0]
        self.joint_pos_max = limits[1]

        # define gain
        self.gain = torch.Tensor([0.26, 0.44, 0.40, 1.11, 1.10, 1.20, 0.85])

        if regularize:
            self.reg_gain = torch.Tensor([5.0, 2.2, 1.3, 0.3, 0.1, 0.1, 0.0])
        else:
            self.reg_gain = torch.Tensor([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def forward(self, state_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        ext = state_dict["motor_torques_external"]

        human_torque = -self.gain * ext

        joint_pos_current = state_dict["joint_positions"]

        left_boundary = 1 / torch.clamp(torch.abs(self.joint_pos_min - joint_pos_current), 1e-8, 100000)
        right_boundary = 1 / torch.clamp(torch.abs(self.joint_pos_max - joint_pos_current), 1e-8, 100000)

        reg_load = left_boundary - right_boundary

        reg_torgue = self.reg_gain * reg_load

        return {"joint_torques": human_torque + reg_torgue}

class ImitationController(toco.policies.HybridJointImpedanceControl):
    def __init__(self, robot: RobotInterface):
        super().__init__(
            joint_pos_current=robot.get_joint_positions(),
            Kq=robot.Kq_default,
            Kqd=robot.Kqd_default,
            Kx=robot.Kx_default,
            Kxd=robot.Kxd_default,
            robot_model=robot.robot_model,
            ignore_gravity=robot.use_grav_comp,
        )

GRIPPER_LOWER_BOUND = 0.01
GRIPPER_UPPER_BOUND = 0.075
GRIPPER_SPEED = 1.0
GRIPPER_FORCE = 20.0
GRIPPER_GRASP_THRESHOLD = 0.05
GRIPPER_OPEN_THRESHOLD = 0.06

class Teleoperation:
    def __init__(self, operated_panda: Panda, imitator_panda: Panda):
        self.logger = spdlog.ConsoleLogger("teleoperation")
        # self.logger.set_level(spdlog.LogLevel.DEBUG)
        self.operated_panda = operated_panda
        self.imitator_panda = imitator_panda

        self.human_controller = HumanController(self.operated_panda.robot)
        self.imitation_controller = ImitationController(self.imitator_panda.robot)

        # Reset imitator to operated robots current positions
        self.logger.info("Setting up imitator")
        self.imitator_panda.robot.move_to_joint_positions(self.operated_panda.robot.get_joint_positions())
        self.imitator_panda.gripper.goto(self.operated_panda.gripper.get_state().width, GRIPPER_SPEED, 0.1)
        self.logger.info("Finished setting up imitator")

        self.logger.info("Sending torch policies")
        self.operated_panda.robot.send_torch_policy(self.human_controller, blocking=False)
        self.imitator_panda.robot.send_torch_policy(self.imitation_controller, blocking=False)
        self.logger.info("Finished sending torch policies")

        self.running = True

        # setup up signal for cleanup
        signal.signal(signal.SIGINT, self.__sig_handler)


    def run(self):
        self.running = True
        last_command = "none"
        last_issued = -1
        while self.running:
            if not self.operated_panda.robot.is_running_policy():
                self.logger.warn(f"Trying to resend {self.operated_panda.name}'s policy")
                self.operated_panda.robot.send_torch_policy(self.human_controller, blocking=False)


            # update robot target position
            joint_pos_desired = self.operated_panda.robot.get_joint_positions()
            try:
                self.imitator_panda.robot.update_desired_joint_positions(joint_pos_desired)
            except:
                self.logger.warn(f"Trying to resend {self.imitator_panda.name}'s policy")
                self.imitator_panda.robot.send_torch_policy(self.imitation_controller, blocking=False)

            # mirror gripper state
            operated_gripper_state = self.operated_panda.gripper.get_state()
            desired_gripper_width = operated_gripper_state.width

            imitator_gripper_state = self.imitator_panda.gripper.get_state()
            imitator_gripper_width = imitator_gripper_state .width

            if imitator_gripper_state.is_moving:
                continue


            if desired_gripper_width < GRIPPER_GRASP_THRESHOLD and not last_command == "grasp":
                self.logger.debug("Grasping")
                self.imitator_panda.gripper.grasp(
                    speed=GRIPPER_SPEED,
                    force=GRIPPER_FORCE,
                    grasp_width=GRIPPER_LOWER_BOUND,
                    epsilon_inner=0.01,
                    epsilon_outer=0.2,
                    blocking=False,
                )

                last_command = "grasp"

            elif desired_gripper_width > GRIPPER_OPEN_THRESHOLD:
                self.logger.debug("Moving")
                self.imitator_panda.gripper.goto(
                        width=GRIPPER_UPPER_BOUND,
                        speed=GRIPPER_SPEED,
                        force=GRIPPER_FORCE,
                        blocking=False,
                )

                last_command = "move"
            

    def __sig_handler(self, _sig, _frame):
        self.running = False

    def cleanup(self):
        self.logger.info("Terminating policies")
        self.operated_panda.robot.terminate_current_policy()
        self.imitator_panda.robot.terminate_current_policy()

