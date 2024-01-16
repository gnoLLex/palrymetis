# Utils

Standard launching for servers is done via polymetis launchers

```sh
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.1.2 port=1234
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.2.2 port=4321
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.1.2 port=1235
launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.2.2 port=4322
```

For ease of use in the lab there is a script to launch them dependent on the robot names

```sh
cd ~/Proj/lengelma/polymetis && ./launch_poly_server.sh <robot, gripper> <p1, p2>
```

or via

```sh
terminator -l PolymetisServers &
```

which is currently only implemented on the P1/P2 Realtime Kernel PC.

```sh
sudo pkill -9 run_server
```

## Asynch Gripper

The asynch implementation actually crashes the gripper server sometimes because of a UDP timeout.
So for now the gripper runs synchronously.

