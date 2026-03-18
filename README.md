# Digital Twin System for a 5-DOF Robotic Arm (ROS2)

## Introduction

This project presents a simulation-driven digital twin of a 5-degree-of-freedom robotic arm developed using ROS2. The system replicates the behavior of a physical robot in a virtual environment and extends beyond simulation by incorporating data recording, monitoring, and anomaly detection.

The robotic arm is equipped with a rotary end-effector, conceptually modeled for drilling operations, enabling the study of constrained motion and operational limits.

---

## Project Focus

Rather than building only a visual simulation, this project emphasizes the complete lifecycle of a digital twin:

* Accurate state representation
* Continuous data capture
* Monitoring system behavior
* Detecting abnormal conditions
* Visualizing system performance

---

## System Workflow

The system is organized as a data-driven pipeline:

Simulation → State Validation → Data Recording → Condition Monitoring → Visualization

Each stage contributes to creating a functional digital twin environment.

---

## Robot Model

* 5 DOF articulated robotic arm
* Rotary gripper (drilling-type end-effector)
* Defined using URDF/Xacro
* Integrated with ros2_control for joint-level control

---

## Implementation Breakdown

### 1. TF and Visualization Stability

* Established a consistent TF tree
* Ensured correct frame transformations for reliable visualization in RViz2

### 2. Joint State Validation

* Verified continuous publishing of `/joint_states`
* Ensured synchronization between simulation and state feedback

### 3. Data Acquisition (rosbag2)

* Recorded joint states and system behavior
* Generated datasets for offline inspection and debugging

### 4. Monitoring and Anomaly Detection

* Implemented a custom ROS2 node to track system constraints
* Detects:

  * Joint limit violations
  * Excessive joint velocities
  * Irregular state transitions

### 5. Data Visualization (Foxglove)

* Integrated Foxglove dashboard for real-time observation
* Enabled monitoring of robot states and detected anomalies

---

## Technology Stack

* ROS2
* Gazebo
* RViz2
* rosbag2
* Foxglove Studio
* URDF / Xacro
* ros2_control

---

## Key Capabilities

* End-to-end digital twin pipeline
* Real-time system monitoring
* Data logging and replay support
* Rule-based anomaly detection
* Dashboard-driven visualization

---

## Practical Relevance

The system aligns with real-world use cases such as:

* Industrial robot monitoring systems
* Predictive maintenance frameworks
* Virtual commissioning and testing
* Remote diagnostics and validation
* Digital twin implementations in smart manufacturing

---

## Strengths of the Approach

* Enables safe testing without physical hardware
* Provides continuous visibility into robot behavior
* Supports debugging through recorded data
* Reduces deployment risks
* Modular and extendable design

---

## Current Constraints

* Limited to simulation environment
* Detection logic is rule-based (not adaptive)
* No real-time feedback loop to control system
* Accuracy depends on simulation parameters

---

## Repository Structure

```id="l9m7op"
digital_twin/
├── config/
│   ├── gazebo_bridge.yaml
│   ├── qos_override.yaml
│   └── ros2_control.yaml
│
├── launch/
│   ├── digital_twin.launch.xml
│   └── gazebo_digital_twin.launch.xml
│
├── pythonfiles/
│   ├── __init__.py
│   └── anomaly_detection.py
│
├── scripts/
│   ├── analyze_bag.py
│   └── record_bag.sh
│
├── rviz/
│   └── rviz_config.rviz
│
├── urdf/
│   ├── body.xacro
│   ├── digital_twin.urdf.xacro
│   └── ros2_control.xacro
│
├── CMakeLists.txt
├── package.xml
└── setup.py
```

---

## Execution Steps

### Build the workspace

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Launch simulation

```
ros2 launch digital_twin gazebo_digital_twin.launch.xml
```

### Record data

```
bash scripts/record_bag.sh
```

### Run anomaly detection

```
ros2 run digital_twin anomaly_detection
```

---

## Future Enhancements

* Integrate machine learning-based anomaly detection
* Connect with real robotic hardware
* Implement feedback control based on detected anomalies
* Enhance dashboards with custom analytics
* Extend to multi-robot digital twin systems

---

## Conclusion

This project demonstrates how a digital twin can be built as a complete system rather than a standalone simulation. By combining data, monitoring, and visualization, it lays the groundwork for intelligent and scalable robotic systems.

---

## Author

Manjunadh
