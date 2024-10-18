from palrymetis import Panda, Recorder
from palrymetis.teleoperation import Teleoperation

if __name__ == "__main__":
    # Initialize robot interface
    operated_panda = Panda.from_alr_name("p3")
    imitator_panda = Panda.from_alr_name("p4")

    teleoperation = Teleoperation(operated_panda, imitator_panda)
    teleoperation.run()

    operated_panda.close()
    imitator_panda.close()

