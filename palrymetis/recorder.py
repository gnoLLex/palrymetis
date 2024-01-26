import threading
import signal
import spdlog
import time

from tqdm import tqdm
from queue import Queue
from typing import Optional, List

from polymetis_pb2 import RobotState
from palrymetis.panda import Panda

ROBOT_STATE_MEMBERS = [
        'joint_positions',
        'joint_torques_computed',
        'joint_velocities',
        'motor_torques_desired',
        'motor_torques_external',
        'motor_torques_measured',
        'prev_command_successful',
        'prev_controller_latency_ms',
        'prev_joint_torques_computed',
        'prev_joint_torques_computed_safened',
        'timestamp'
]

GRIPPER_STATE_MEMBERS = [
        'error_code',
        'is_grasped',
        'is_moving',
        'prev_command_successful',
        'timestamp',
        'width'
]

# TODO: gripper state subscriptions
class Recorder:
    def __init__(
        self,
        robot: Panda,
        running: callable,
        subscriptions: Optional[List[str]] = None,
        hz: float = 10.0
    ):
        """
        """
        self.logger = spdlog.ConsoleLogger("recorder")
        self.robot = robot
        self.recordings = {}
        self.thread = None
        self.running = running
        self.hz = hz
        
        subscriptions = subscriptions if subscriptions else ROBOT_STATE_MEMBERS

        for sub in subscriptions:
            if not sub in ROBOT_STATE_MEMBERS: # and not sub in GRIPPER_STATE_MEMBERS 
                self.logger.error(f"No subscriptable member called {sub} found.\nTry {ROBOT_STATE_MEMBERS}") # or {GRIPPER_STATE_MEMBERS}")

        self.subscriptions = subscriptions

        for sub in self.subscriptions:
            self.recordings[sub] = []

    def start(self):
        """
        """
        self.thread = threading.Thread(target=self._record)
        self.thread.start()

    def _record(self):
        last_timestamp = 0
        while self.running():
            state = self.robot.robot.get_robot_state()

            # skip if we receive the same state
            if not last_timestamp == state.timestamp:
                for sub in self.subscriptions:
                    self.recordings[sub].append(getattr(state, sub))
                time.sleep(1 / self.hz)

            last_timestamp = state.timestamp

    def stop(self):
        """
        """
        if self.thread:
            self.thread.join()
            self.thread = None

    def save(self):
        """
        """
        import os
        import pandas
        from datetime import datetime

        save_dir = "outputs/recordings/"

        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
    
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        filename = f"{timestamp}.csv"
        full_path = os.path.join(save_dir, filename)

        self.logger.info(f"Saving to {full_path}")
        if 'timestamp' in self.subscriptions:
            self.recordings['timestamp'] = [ts.seconds * 1e9 + ts.nanos for ts in self.recordings['timestamp']] 
        pandas.DataFrame(self.recordings).to_csv(full_path, float_format='%32.f')

