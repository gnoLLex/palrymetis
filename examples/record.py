from palrymetis import Panda, Recorder
from palrymetis.controllers import HumanController

import time

if __name__ == "__main__":
    panda = Panda.from_alr_name("p2")
    panda.load_policy(HumanController(panda.robot), blocking=False)

    running = True
    recorder = Recorder(
            panda,
            lambda: running,
            ["joint_positions", "joint_velocities"],
            hz=1000
    )

    while running:
        char = input()
        if char == "q":
            running = False
        elif char == "r":
            recorder.start()
        elif char == "s":
            recorder.stop()

    recorder.save()
    panda.close()
