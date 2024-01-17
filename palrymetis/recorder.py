import threading
import signal
import spdlog

from tqdm import tqdm
from queue import Queue

from palrymetis.panda import Panda

class Recorder:
    def __init__(self, robot: Panda, running):
        self.logger = spdlog.ConsoleLogger("recorder")
        self.robot = robot
        self.recordings = []
        self.thread = None
        self.running = running

    def start(self):
        self.thread = threading.Thread(target=self._record)
        self.thread.start()

    def _record(self):
        while self.running():
            self.recordings.append(self.robot.arm.get_robot_state())

    def stop(self):
        if self.thread:
            self.thread.join()
            self.thread = None

    # TODO: proper fileformat and subscribtable properties
    def save(self):
        import os
        import json
        from datetime import datetime

        save_dir = "outputs/recordings/"

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
    
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        filename = f"{timestamp}.txt"
        full_path = os.path.join(save_dir, filename)

        self.logger.info(f"Saving to {full_path}")
        with open(full_path, 'w') as f:
            for recording in tqdm(self.recordings):
                f.write(f"{recording.timestamp}_{recording.joint_positions}\n")
