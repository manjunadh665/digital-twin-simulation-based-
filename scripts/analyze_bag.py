#!/usr/bin/env python3
import warnings
warnings.filterwarnings('ignore')
import numpy as np
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
import sys
import os

def analyze_bag(bag_path):
    print(f'Analyzing: {bag_path}')

    typestore = get_typestore(Stores.LATEST)  # changed from ROS2_HUMBLE

    timestamps = []
    joint_data = {
        'first_joint':   [],
        'second_joint':  [],
        'third_joint':   [],
        'fourth_joint':  [],
        'gripper_joint': []
    }
    velocities = {j: [] for j in joint_data}

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections
                      if c.topic == '/joint_states']

        for connection, timestamp, rawdata in reader.messages(
                connections=connections):
            msg = typestore.deserialize_cdr(rawdata, connection.msgtype)    

            t = timestamp / 1e9
            timestamps.append(t)

            for i, name in enumerate(msg.name):
                if name in joint_data:
                    joint_data[name].append(msg.position[i])
                    velocities[name].append(msg.velocity[i])

    if not timestamps:
        print('No joint_states data found!')
        return

    t0 = timestamps[0]
    timestamps = [t - t0 for t in timestamps]

    print(f'Duration:  {timestamps[-1]:.1f} seconds')
    print(f'Messages:  {len(timestamps)}')

    # Plot
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle('Digital Twin — Joint Analysis', fontsize=14)

    colors = ['blue', 'orange', 'green', 'red', 'purple']

    # Position graph
    ax1 = axes[0]
    for idx, (joint, positions) in enumerate(joint_data.items()):
        if positions:
            ax1.plot(timestamps, positions,
                    label=joint, color=colors[idx], linewidth=1.5)
    ax1.set_title('Joint Positions Over Time')
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Position (radians)')
    ax1.legend()
    ax1.grid(True)

    # Velocity graph
    ax2 = axes[1]
    for idx, (joint, vels) in enumerate(velocities.items()):
        if vels:
            ax2.plot(timestamps, vels,
                    label=joint, color=colors[idx], linewidth=1.5)
    ax2.set_title('Joint Velocities Over Time')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.axhline(y=2.0,  color='red', linestyle='--', label='max velocity')
    ax2.axhline(y=-2.0, color='red', linestyle='--')
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()

    # Save
    output_path = bag_path + '_analysis.png'
    plt.savefig(output_path, dpi=150)
    print(f'\nPlot saved: {output_path}')
    plt.show()

    # Statistics
    print('\n--- Joint Statistics ---')
    for joint, positions in joint_data.items():
        if positions:
            print(f'\n{joint}:')
            print(f'  min:  {min(positions):.3f} rad')
            print(f'  max:  {max(positions):.3f} rad')
            print(f'  mean: {np.mean(positions):.3f} rad')
            print(f'  std:  {np.std(positions):.3f} rad')

if __name__ == '__main__':
    if len(sys.argv) > 1:
        bag_path = sys.argv[1]
    else:
        bag_dir = os.path.expanduser('~/arm_bags')
        bags = sorted(os.listdir(bag_dir))
        if not bags:
            print('No bags found in ~/arm_bags/')
            sys.exit(1)
        bag_path = os.path.join(bag_dir, bags[-1])
        print(f'Using latest bag: {bag_path}')

    analyze_bag(bag_path)