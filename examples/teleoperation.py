from palrymetis.teleoperation import Teleoperation
from palrymetis.panda import Panda

if __name__ == "__main__":
    # Initialize robot interface
    operated_panda = Panda.from_alr_name("p2")
    imitator_panda = Panda.from_alr_name("p1")

    teleoperation = Teleoperation(operated_panda, imitator_panda)
    teleoperation.run() # runs until ctrl-c
    teleoperation.cleanup()

