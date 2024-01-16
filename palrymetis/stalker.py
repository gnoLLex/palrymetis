from queue import Queue
import threading
import matplotlib.pyplot as plt

class Stalker:
    def __init__(self):
        pass

def stalker_function(victim, queue):
    num_joints = 7
    jpo = [[] for _ in range(num_joints)]
    jpi = [[] for _ in range(num_joints)]
    jpd = [[] for _ in range(num_joints)]
    jvo = [[] for _ in range(num_joints)]
    jvi = [[] for _ in range(num_joints)]
    jvd = [[] for _ in range(num_joints)]
    tso = []
    tsi = []
    while victim.running:
        # plotting
        state_operated = victim.operated_robot.get_robot_state()
        state_imitator = victim.imitator_robot.get_robot_state()
        joint_pos_operated = state_operated.joint_positions
        joint_pos_imitator = state_imitator.joint_positions
        joint_vel_operated = state_operated.joint_velocities
        joint_vel_imitator = state_imitator.joint_velocities
        timestamp_operated = state_operated.timestamp
        timestamp_imitator = state_imitator.timestamp
        
        for joint in range(num_joints):
            jpo[joint].append(joint_pos_operated[joint])
            jpi[joint].append(joint_pos_imitator[joint])
            jpd[joint].append(abs(joint_pos_operated[joint] - joint_pos_imitator[joint]))
        
            jvo[joint].append(joint_vel_operated[joint])
            jvi[joint].append(joint_vel_imitator[joint])
            jvd[joint].append(abs(joint_vel_operated[joint] - joint_vel_imitator[joint]))
        
        tso.append(timestamp_imitator.ToDatetime())
        tsi.append(timestamp_imitator.ToDatetime())
    
    queue.put((jpo, jpi, jpd, jvo, jvi, jvd, tso, tsi))




def plot_pos(jpo, jpi, jpd, tso, tsi):
    num_joints = 7
    rows, cols = 4, 2
    fig, axs = plt.subplots(cols, rows, figsize=(10, 8))
    fig.suptitle('Comparison of Joint Angles over Time')
    
    limits = operated_robot.robot_model.get_joint_angle_limits()

    for joint in range(num_joints):
        i, j = joint//rows, joint%rows
        axs[i][j].plot(tso, jpo[joint], label='Operated')
        axs[i][j].plot(tsi, jpi[joint], label='Imitator')
        axs[i][j].set_ylim(limits[0][joint], limits[1][joint])
        axs[i][j].set_xticks([])
        axs[i][j].set_title(f'Joint {joint + 1} Position')
        axs[i][j].legend()

        axs_c = axs[i][j].twinx()

        axs_c.plot(tso, jpd[joint], color=(0.3, 0.5, 0.3, 0.5), label=f'Absolute Difference')
        axs_c.set_ylim(0, 1)
    
    for joint in range(num_joints):
        axs[cols-1][rows-1].plot(tso, jpd[joint], alpha=0.5, label=f'Joint {joint + 1}')
    axs[cols-1][rows-1].set_xticks([])
    axs[cols-1][rows-1].set_title(f'Position Differences')
    axs[cols-1][rows-1].legend()
    

def plot_vel(jvo, jvi, jvd, tso, tsi):
    num_joints = 7
    rows, cols = 4, 2
    fig, axs = plt.subplots(cols, rows, figsize=(10, 8))
    fig.suptitle('Comparison of Joint Velocites over Time')
    
    limits = operated_robot.robot_model.get_joint_velocity_limits()

    for joint in range(num_joints):
        i, j = joint//rows, joint%rows
        axs[i][j].plot(tso, jvo[joint], label='Operated')
        axs[i][j].plot(tsi, jvi[joint], label='Imitator')
        axs[i][j].set_ylim(-limits[joint], limits[joint])
        axs[i][j].set_xticks([])
        axs[i][j].set_title(f'Joint {joint + 1} Velocity')
        axs[i][j].legend()

        axs_c = axs[i][j].twinx()

        axs_c.plot(tso, jvd[joint], color=(0.3, 0.5, 0.3, 0.5), label=f'Absolute Difference')
        axs_c.set_ylim(0, 1)
    
    for joint in range(num_joints):
        axs[cols-1][rows-1].plot(tso, jpd[joint], alpha=0.5, label=f'Joint {joint + 1}')
    axs[cols-1][rows-1].set_xticks([])
    axs[cols-1][rows-1].set_title(f'Velocity Differences')
    axs[cols-1][rows-1].legend()

    # stalker.join()
    #queue = Queue()
    # stalker = threading.Thread(target=stalker_function, args=(teleoperation, queue))

    # stalker.start()

    # jpo, jpi, jpd, jvo, jvi, jvd, tso, tsi = queue.get()
    # plot_pos(jpo, jpi, jpd, tso, tsi)
    # plot_vel(jvo, jvi, jvd, tso, tsi)
    # plt.show()