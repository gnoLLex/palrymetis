from polymetis import RobotInterface, GripperInterface
from palrymetis.teleoperation import Teleoperation

if __name__ == "__main__":
    # Initialize robot interface
    operated_robot = RobotInterface(
        ip_address="localhost",
        port="4321"
    )
    imitator_robot = RobotInterface(
        ip_address="localhost",
        port="1234"
    )
    operated_robot_hand = GripperInterface(
        ip_address="localhost",
        port="4322"
    )
    imitator_robot_hand = GripperInterface(
        ip_address="localhost",
        port="1235"
    )

    teleoperation = Teleoperation(operated_robot, operated_robot_hand, imitator_robot, imitator_robot_hand)
    teleoperation.run() # runs until ctrl-c
    teleoperation.cleanup()

