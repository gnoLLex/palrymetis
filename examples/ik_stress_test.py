from polymetis import RobotInterface
import numpy as np
import torch


test_ee_poses = [
    #### front left bottom corner
    ([0.6, -0.25, 0.15], [0.009, 0.72, -0.67, -0.014]),
    ([0.6, -0.25, 0.15], [0.7071, 0.7071, 0, 0]),
    ([0.6, -0.25, 0.15], [0.7071, 0.7071, 0, 0]),

    #### front right bottom corner
    ([0.6, 0.25, 0.15], [0.009, 0.72, -0.67, -0.014]),
    ([0.6, 0.25, 0.15], [0.7071, 0.7071, 0, 0]),
    ([0.6, 0.25, 0.15], [-0.7071, 0.7071, 0, 0]),

    #### front right top corner
    ([0.6, 0.25, 0.65], [0.009, 0.72, -0.67, -0.014]),
    ([0.6, 0.25, 0.65], [0.7071, 0.7071, 0, 0]),
    ([0.6, 0.25, 0.65], [-0.7071, 0.7071, 0, 0]),

    #### front left top corner
    ([0.6, -0.25, 0.65], [0.009, 0.72, -0.67, -0.014]),
    ([0.6, -0.25, 0.65], [0.7071, 0.7071, 0, 0]),
    ([0.6, -0.25, 0.65], [-0.7071, 0.7071, 0, 0]),

    #### back left bottom corner
    ([0.35, -0.25, 0.15], [0.009, 0.72, -0.67, -0.014]),
    ([0.35, -0.25, 0.15], [0.7071, 0.7071, 0, 0]),
    ([0.35, -0.25, 0.15], [-0.7071, 0.7071, 0, 0]),

    #### back right bottom corner
    ([0.35, 0.25, 0.15], [0.009, 0.72, -0.67, -0.014]),
    ([0.35, 0.25, 0.15], [0.7071, 0.7071, 0, 0]),
    ([0.35, 0.25, 0.15], [-0.7071, 0.7071, 0, 0]),

    #### back right top corner
    ([0.35, 0.25, 0.45], [0.009, 0.72, -0.67, -0.014]),
    ([0.35, 0.25, 0.45], [0.7071, 0.7071, 0, 0]),
    ([0.35, 0.25, 0.45], [-0.7071, 0.7071, 0, 0]),

    #### back left top corner
    ([0.35, -0.25, 0.45], [0.009, 0.72, -0.67, -0.014]),
    ([0.35, -0.25, 0.45], [0.7071, 0.7071, 0, 0]),
    ([0.35, -0.25, 0.45], [-0.7071, 0.7071, 0, 0]),
]

if __name__ == "__main__":

    robot = RobotInterface(ip_address="localhost", port=50051)

    invalid_poses = 0
    for pos, ori in test_ee_poses:
        print(pos, ori)
        state = robot.move_to_ee_pose(
            position=torch.Tensor(pos),
            orientation=torch.Tensor(ori),
            time_to_go=1.0
        )
        if state == []:
            invalid_poses += 1

    print(f"Invalid Poses: {invalid_poses}/{len(test_ee_poses)} | {(invalid_poses / len(test_ee_poses) * 100):03.2f}%")
    robot.move_to_ee_pose(position=torch.Tensor([5.50899712e-01, -1.03382391e-08, 6.5e-01]), orientation=[0, 1, 0, 0], time_to_go=2.0)
