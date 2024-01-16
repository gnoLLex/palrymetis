import torch
import spdlog
import signal

import torchcontrol as toco

from typing import Dict

class HumanController(toco.PolicyModule):
    # stolen from SimulationFramework
    def __init__(self, robot, regularize=True):
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
    def __init__(self, robot):
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
GRIPPER_UPPER_BOUND = 0.07
GRIPPER_SPEED = 10.0
GRASP_THRESHOLD = 0.03
GRASP_FORCE = 20.0

class Teleoperation:
    def __init__(self, operated_robot, operated_hand, imitator_robot, imitator_hand):
        self.logger = spdlog.ConsoleLogger("teleoperation")
        self.operated_robot = operated_robot
        self.imitator_robot = imitator_robot
        self.operated_hand = operated_hand
        self.imitator_hand = imitator_hand

        self.human_controller = HumanController(self.operated_robot)
        self.imitation_controller = ImitationController(self.imitator_robot)

        # Reset imitator to operated robots current positions
        self.logger.info("Setting up imitator")
        self.imitator_robot.move_to_joint_positions(self.operated_robot.get_joint_positions())
        self.imitator_hand.goto(self.operated_hand.get_state().width, GRIPPER_SPEED, 0.1)
        self.logger.info("Finished setting up imitator")

        self.logger.info("Sending torch policies")
        self.operated_robot.send_torch_policy(self.human_controller, blocking=False)
        self.imitator_robot.send_torch_policy(self.imitation_controller, blocking=False)
        self.logger.info("Finished sending torch policies")

        # setup up signal for cleanup
        signal.signal(signal.SIGINT, self.__sig_handler)


    def run(self):
        self.running = True
        last_command = "none"
        last_issued = 10
        while self.running:
            # update arm target position
            joint_pos_desired = self.operated_robot.get_joint_positions()
            self.imitator_robot.update_desired_joint_positions(joint_pos_desired)

            # mirror gripper state
            operated_hand_state = self.operated_hand.get_state()
            desired_gripper_width = operated_hand_state.width


            # don not send a new gripper command if the desired width is already reached (within margin)
            if abs(desired_gripper_width - last_issued) < 0.005:
                continue


            if desired_gripper_width < GRASP_THRESHOLD and not last_command == "grasp": # or last_issued < desired_gripper_width):
                self.logger.info("Grasping")
                self.imitator_hand.grasp(
                    speed=GRIPPER_SPEED,
                    force=GRASP_FORCE,
                    grasp_width=desired_gripper_width,
                    epsilon_inner=0.01,
                    epsilon_outer=1,
                )
                last_command = "grasp"

            elif desired_gripper_width > GRASP_THRESHOLD:
                self.logger.info("Moving")
                self.imitator_hand.goto(desired_gripper_width, GRIPPER_SPEED, 5)
                last_command = "move"
            
            last_issued = desired_gripper_width

    def __sig_handler(self, _sig, _frame):
        self.running = False

    def cleanup(self):
        self.logger.info("Terminating policies")
        self.operated_robot.send_torch_policy(self.human_controller, blocking=False)
        self.operated_robot.terminate_current_policy()
        self.imitator_robot.terminate_current_policy()

