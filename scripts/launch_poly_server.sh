#!/bin/bash

try_activate_env() {
    if [[ "$CONDA_DEFAULT_ENV" != "$1" ]]
    then
        echo Activating $1 env
    
        eval "$(conda shell.bash hook)"
        conda activate $1
    
        if [[ "$CONDA_DEFAULT_ENV" != "$1" ]]
        then
            echo Failed to activate conda environment.
            exit 1
        fi
    fi
}

get_robot_attr() {
	python3 -c "import sys, yaml; print(yaml.load(open(\"configs/robots.yaml\"), yaml.FullLoader)[\"$1\"][\"$2\"])"
}


try_activate_env polymetis

if [[ $1 == "robot" ]]; then
    launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=$(get_robot_attr $2 ip) port=$(get_robot_attr $2 port)

    if [ $? -eq 0 ]; then
        echo "Command was successful"
    else
        hash_line="##################################################################################"
        echo -e "\e[1;31m$hash_line\e[0m"
        echo "Command failed with exit code $?"
        echo "Make sure Panda 1 is in FCI mode and make sure no other server is by running:"
        echo "sudo pkill -9 run_server"
        echo -e "\e[1;31m$hash_line\e[0m"
    fi
elif [[ $1 == "gripper" ]]; then
    launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=$(get_robot_attr $2 ip) port=$(get_robot_attr $2 gripper_port)
fi
