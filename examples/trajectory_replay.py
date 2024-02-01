import torchcontrol as toco
from torchcontrol.policies import JointTrajectoryExecutor
from torchcontrol.utils.tensor_utils import to_tensor, stack_trajectory
from palrymetis import Panda
from typing import List, Dict

import torch
import ast
import time
import sys
import tqdm


if __name__ == "__main__":
    panda = Panda.from_alr_name("p2")

    
    def load_traj(filename: str):
        import os
        import pandas
        from datetime import datetime

        data = dict()

        csv = pandas.read_csv(filename, index_col=False)
        for key in tqdm.tqdm(csv.columns.values):
            if not key == "joint_positions" and not key == "joint_velocities":
                continue
            if key == "timestamp":
                data[key] = [val for val in csv[key]]
                continue
            data[key] = [torch.Tensor(val) for val in csv[key].apply(ast.literal_eval).to_list()]

        return data

    traj = load_traj(sys.argv[1])

    policy = toco.policies.JointTrajectoryExecutor(
        joint_pos_trajectory=traj["joint_positions"],
        joint_vel_trajectory=traj["joint_velocities"],
        Kq=panda.robot.Kq_default,
        Kqd=panda.robot.Kqd_default,
        Kx=panda.robot.Kx_default,
        Kxd=panda.robot.Kxd_default,
        robot_model=panda.robot.robot_model,
        ignore_gravity=panda.robot.use_grav_comp,
    )

    panda.robot.move_to_joint_positions(traj["joint_positions"][0])
    panda.load_policy(policy)


