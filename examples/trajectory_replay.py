import torchcontrol as toco
from torchcontrol.policies import JointTrajectoryExecutor
from torchcontrol.utils.tensor_utils import to_tensor, stack_trajectory
from palrymetis import Panda
from typing import List, Dict

import torch
import ast
import time

class ContinuousJointTrajectoryExecutor(toco.PolicyModule):
    def __init__(
        self,
        joint_pos_trajectory: List[torch.Tensor],
        joint_vel_trajectory: List[torch.Tensor],
        Kq,
        Kqd,
        Kx,
        Kxd,
        robot_model: torch.nn.Module,
        ignore_gravity=True,
    ):
        super().__init__()

        self.joint_pos_trajectory = torch.nn.Parameter(to_tensor(stack_trajectory(joint_pos_trajectory)))
        self.joint_vel_trajectory = torch.nn.Parameter(to_tensor(stack_trajectory(joint_vel_trajectory)))
        self.terminate = torch.nn.Parameter(False)
        self.i = torch.nn.Parameter(0)

        self.N = 1000
        assert self.joint_pos_trajectory.shape == self.joint_vel_trajectory.shape

        # Control modules
        self.robot_model = robot_model
        self.invdyn = toco.modules.feedforward.InverseDynamics(
            self.robot_model, ignore_gravity=ignore_gravity
        )
        self.joint_pd = toco.modules.feedback.HybridJointSpacePD(Kq, Kqd, Kx, Kxd)


    def forward(self, state_dict: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        if self.i >= self.N:
            return {"joint_torques": torch.Tensor([0, 0, 0, 0, 0, 0, 0])}
        # Parse current state
        joint_pos_current = state_dict["joint_positions"]
        joint_vel_current = state_dict["joint_velocities"]

        # Query plan for desired state
        joint_pos_desired = self.joint_pos_trajectory[self.i, :]
        joint_vel_desired = self.joint_vel_trajectory[self.i, :]

        # Control logic
        torque_feedback = self.joint_pd(
            joint_pos_current,
            joint_vel_current,
            joint_pos_desired,
            joint_vel_desired,
            self.robot_model.compute_jacobian(joint_pos_current),
        )
        torque_feedforward = self.invdyn(
            joint_pos_current, joint_vel_current, torch.zeros_like(joint_pos_current)
        )  # coriolis
        torque_out = torque_feedback + torque_feedforward

        # Increment & termination
        self.i += 1

        return {"joint_torques": torque_out}


if __name__ == "__main__":
    panda = Panda.from_alr_name("p2")

    
    def load_traj(filename: str):
        import os
        import pandas
        from datetime import datetime

        save_dir = "outputs/recordings/"

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
    
        full_path = os.path.join(save_dir, filename)

        data = dict()

        csv = pandas.read_csv(full_path, index_col=False)
        for key in csv.columns.values:
            if key == "timestamp":
                data[key] = [val for val in csv[key]]
                continue
            data[key] = [torch.Tensor(val) for val in csv[key].apply(ast.literal_eval).to_list()]

        return data

    traj = load_traj("20240131121825.csv")
    waypoints = {"pos": [], "vel": []}
    for i in range(len(traj["joint_positions"]) - 1):
        pos = traj["joint_positions"][i]
        pos_next = traj["joint_positions"][i + 1]
        waypoint = toco.planning.generate_joint_space_min_jerk(
            start=pos,
            goal=pos_next,
            time_to_go=0.9,
            hz=panda.robot.metadata.hz,
        )
        for w in waypoint:
            waypoints["pos"].append(w["position"])
            waypoints["vel"].append(w["velocity"])


    #policy = toco.policies.JointTrajectoryExecutor(
    #    joint_pos_trajectory=waypoints["pos"][:1000],
    #    joint_vel_trajectory=waypoints["vel"][:1000],
    #    Kq=panda.robot.Kq_default,
    #    Kqd=panda.robot.Kqd_default,
    #    Kx=panda.robot.Kx_default,
    #    Kxd=panda.robot.Kxd_default,
    #    robot_model=panda.robot.robot_model,
    #    ignore_gravity=panda.robot.use_grav_comp,
    #)

    #panda.robot.move_to_joint_positions(traj["joint_positions"][0])
    #panda.robot.send_torch_policy(policy)
    #time.sleep(2)
    #for i in range(len(traj["joint_positions"]) - 1):
    #    panda.robot.update_current_policy({
    #        "joint_pos_trajectory": to_tensor(stack_trajectory(waypoints["pos"][i*1000:i*1000 + 1000])),
    #        "joint_vel_trajectory": to_tensor(stack_trajectory(waypoints["vel"][i*1000:i*1000 + 1000])),
    #        "terminate": torch.Tensor([False]),
    #        "i": torch.Tensor(0),
    #    })
    #    time.sleep(2)


    panda.robot.move_to_joint_positions(traj["joint_positions"][0])
    panda.robot.start_joint_impedance()

    for i in range(len(traj["joint_positions"]) - 1):
        pos = traj["joint_positions"][i]
        time_delta = (traj["timestamp"][i+1] - traj["timestamp"][i]) / 1e9

        panda.robot.update_desired_joint_positions(pos)
        time.sleep(time_delta - 0.01)

        
    panda.robot.update_desired_joint_positions(traj["joint_positions"][-1])
    panda.robot.terminate_current_policy()


