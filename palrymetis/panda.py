from polymetis import RobotInterface, GripperInterface

class Panda:
    def __init__(self, ip, arm_port, gripper_port):
        self.arm = RobotInterface(
            ip_address=ip,
            port=arm_port
        )
        self.gripper = GripperInterface(
            ip_address=ip,
            port=gripper_port
        )

    @classmethod
    def from_alr_name(cls, name):
        import sys, yaml
        # TODO: config path (maybe env var?)
        panda_config = yaml.load(open("configs/robots.yaml"), yaml.FullLoader)[name]

        ip = "localhost" # TODO: localhost for now, ip of rt PC otherwise

        panda_arm_port = panda_config["port"]
        panda_gripper_port = panda_config["gripper_port"]

        return cls(ip, panda_arm_port, panda_gripper_port)

