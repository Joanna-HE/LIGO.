# LIGO 

**LIGO: Tightly Coupled LiDAR-Inertial-GNSS Odometry based on a Hierarchy Fusion Framework for Global Localization with Real-time Mapping**

Code, paper, video are coming soon......

# Developers:
The codes of this repo are contributed by:
[Dongjiao He (贺东娇)](https://github.com/Joanna-HE)

# Properties

**LIGO is a multi-sensor fusion framework that maximizes the complementary properties of both LiDAR and GNSS systems**. This package achieves the following properties:

1. Competitive accuracy in trajectory estimation across large-scale scenarios.
2. Robustness to malfunctions of either GNSS or LiDAR sensors, enabling seamless handling of added or lost sensor signals during operation.
3. High-output-frequency odometry.
4. Capability of providing globally referenced pose estimations in both indoor and outdoor environments, suitable for ground vehicles and uncrewed aerial vehicles (UAVs).
5. No requirement for GNSS observations to be obtained exactly at the beginning or end time of LiDAR scans.
6. Robustness to large outliers and high noise levels in GNSS observations.

# Build

## Prerequisites

### C++14 Compier

### ROS noetic

### Eigen 3

### GTSAM

### [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) with its [instuction](https://github.com/HKUST-Aerial-Robotics/gnss_comm#2-build-gnss_comm-library)

## Make

```cd ~/catkin_ws/src/
git clone https://github.com/Joanna-HE/LIGO..git

```cd ~/catkin_ws/
source /PATH/TO/GNSS_COMM/DEVEL/.setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash'''

# Demo
**Performance on a sequence with severe LiDAR degeneracy**

<div align="center">
    <div align="center">
        <img src="https://github.com/Joanna-HE/LIGO/blob/main/image/Sample.png" width = 75% >
    </div>
</div>
