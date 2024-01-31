from palrymetis import Teleoperation, Panda, Recorder

if __name__ == "__main__":
    # Initialize robot interface
    operated_panda = Panda.from_alr_name("p2")
    imitator_panda = Panda.from_alr_name("p1")

    teleoperation = Teleoperation(operated_panda, imitator_panda)
    recorder = Recorder(
            imitator_panda,
            lambda: teleoperation.running,
            ['timestamp', 'joint_positions', 'joint_velocities'],
            hz=10
    )
    recorder.start()
    teleoperation.run()
    recorder.stop()

    teleoperation.cleanup()
    recorder.cleanup()

    recorder.save()

