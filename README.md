# LIGO 

**LIGO: Tightly Coupled LiDAR-Inertial-GNSS Odometry based on a Hierarchy Fusion Framework for Global Localization with Real-time Mapping**

Please check the issue created by me (Joanna He), it may answer your questions.
Instruction for using will be detailed soon

Our paper is published on [TRO](https://github.com/Joanna-HE/LIGO./blob/main/paper/LIGO_A_Tightly_Coupled_LiDAR-Inertial-GNSS_Odometry_Based_on_a_Hierarchy_Fusion_Framework_for_Global_Localization_With_Real-Time_Mapping.pdf)

Our datasets are uploaded on [Google Drive](https://drive.google.com/drive/folders/1hNwl8u8Pg-SqKh2N808XFixj6PjPf091?usp=sharing)

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

**Sorry there are so many parameters to be adjusted due to the fusion at a raw level**

# Hardware setups for self-collected datasets

## Setup

<div align="left">
    <div align="left">
        <img src="https://github.com/Joanna-HE/LIGO./blob/main/image/hardware.jpg" width = 30% >
    </div>
</div>

Platform: DJI Matrice 300   
Onboard computer: DJI Manifold 2-c 256G, CPU: Intel i7-8550U  
LiDAR: Livox Mid360 and Livox Avia  
IMU: Built-in IMU of Livox LiDAR  
GNSS receiver: u-blox C099-F9P-2  
GNSS antenna: B4QA4GGGB  

## Recording rates

LiDAR: 10Hz  
IMU: 200Hz  
GNSS: 10Hz  
RTK: 10Hz  

## Recording software:

Operating system: Ubuntu 20.04  
IMU and LiDAR driver: Livox driver  
GNSS driver: [ublox driver](https://github.com/Joanna-HE/ublox_driver)  

## ROS topics recorded

IMU: /livox/imu  
LiDAR: /livox/lidar  
RAW GNSS: /ublox_driver/range_meas  
GNSS EPHEM: /ublox_driver/ephem and /ublox_driver/glo_ephem  
IONO PARAMETER: /ublox_driver/iono_params  
Onboard pos solution of ublox: /ublox_driver/receiver_pvt and /ublox_driver/receiver_lla  
PPS time info: /ublox_driver/time_pulse_info  

## Time synchronization

PPS: Livox LiDARs can receive pps and gprmc given by the GNSS receiver  
The time difference between LiDAR and IMU is zero, and between LiDAR and GNSS message is 18.0 s  

## RTK solution

Please follow the *Differential GNSS* section shown in [ublox driver](https://github.com/Joanna-HE/ublox_driver) to get the differential GNSS solution online or offline. The self-collected datasets get the online RTK solution which are saved in the topic '/ublox_driver/receiver_pvt', the value of the 'carr_soln' as 1 and 'diff_soln' as 2 indicates the fix RTK solution.

# Build

## Option A: Docker (recommended)

The provided image bundles ROS Noetic and every dependency (Ceres 2.1, GTSAM 4.0, Sophus, Livox SDK + `livox_ros_driver`, `gnss_comm`).

```bash
cd LIGO.
docker build -f docker/Dockerfile -t ligo:noetic .      # add --build-arg JOBS=N to control parallelism

# Launch LIGO + RViz (uses host networking, so `rosbag play` on the host works directly)
docker/run.sh roslaunch ligo mapping_avia.launch

# Or open a shell with a folder of bags mounted read-only at /data
DATA_DIR=~/bags docker/run.sh bash
#   (inside) roslaunch ligo mapping_avia.launch &  rosbag play /data/xxx.bag
```

Trajectories (`Log/`) and saved maps (`PCD/`) are written to `./output` on the host.
If RViz cannot open a window, run `xhost +local:` on the host first.

## Option B: Native build

### Prerequisites

Tested on Ubuntu 20.04 with ROS Noetic, a C++17 compiler, and:

| Dependency | Version | Notes |
|---|---|---|
| Eigen | 3.3+ | `sudo apt install libeigen3-dev` |
| PCL | 1.10 | ships with ROS Noetic |
| Ceres Solver | **2.1+** | the apt package on 20.04 (1.14) is too old; build from source |
| GTSAM | 4.0.x | e.g. `ppa:borglab/gtsam-release-4.0` |
| Sophus | any | `sudo apt install ros-noetic-sophus` |
| Boost | | `sudo apt install libboost-all-dev` |
| [Livox ROS driver](https://github.com/Livox-SDK/livox_ros_driver) | | requires [Livox-SDK](https://github.com/Livox-SDK/Livox-SDK) |
| [gnss_comm](https://github.com/HKUST-Aerial-Robotics/gnss_comm) | | see its [instructions](https://github.com/HKUST-Aerial-Robotics/gnss_comm#2-build-gnss_comm-library) |

### Compile

```bash
cd ~/catkin_ws/src/
git clone https://github.com/Joanna-HE/LIGO..git
cd ~/catkin_ws/
source /PATH/TO/LIVOX_DRIVER/devel/setup.bash
source /PATH/TO/GNSS_COMM/devel/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

# Demo
**Performance on a sequence with severe LiDAR degeneracy**

<div align="center">
    <div align="center">
        <img src="https://github.com/Joanna-HE/LIGO/blob/main/image/Sample.png" width = 75% >
    </div>
</div>
