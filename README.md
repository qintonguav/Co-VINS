# Co-VINS
## Collaborative Localization for Multiple Monocular Visual-Inertial Systems

Co-VINS a collaborative localization framework for multiple robots using monocular visual-inertial systems (VINS). Unlike traditional swarm applications which rely on external position equipment (GPS or Motion Capture System), our system achieves globally consistent localization based on internal sensors (onboard camera and IMU). Each robot is equipped with one camera and one IMU. It estimates own pose onboard and sends visual information to a centralized ground station. The ground station collects collaborative information from all robots, and maintains a globally consistent coordinate by pose graph optimization. Then the global localization is feedbacked to each robot for global control purpose. The ground station not only aligns all robots in a global coordinate, but also correct accumulated drifts for each robot. Co-VINS is an extension of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).


**Authors:** [Tong Qin](http://www.qintonguav.com/), [William Wu](https://github.com/justwillim),  and [Shaojie Shen](http://www.ece.ust.hk/ece.php/profile/facultydetail/eeshaojie) from [HUKST Aerial Robotics Group](http://uav.ust.hk/)

**Videos:**

<a href="https://www.youtube.com/embed/OPahuRQH7-8" target="_blank"><img src="http://img.youtube.com/vi/OPahuRQH7-8/0.jpg" 
alt="Swarm application" width="240" height="180" border="10" /></a>

**Related Papers**
* **Technical report: Collaborative Localization for Multiple Monocular Vision-Based MAVs**, Tong Qin, William Wu, Shaojie Shen [Technical report](https://github.com/qintonguav/Co-VINS/blob/vins_swarm/support_files/paper/co_vins_report.pdf) 


## 1. Prerequisites 
1.1 **Ubuntu** and **ROS**
Ubuntu  16.04.
ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)
additional ROS pacakge
```
    sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```


1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.
(Our testing environment: Ubuntu 16.04, ROS Kinetic, OpenCV 3.3.1, Eigen 3.3.3) 

## 2. Build Co-VINS on ROS
Clone the repository and catkin_make: (If [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) exists in your workspace, please remove it first)
```
    cd ~/catkin_ws/src
    git clone https://github.com/qintonguav/Co-VINS.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Run datasets
Use your own computer as both multiple robots and ground station. Run several VIOs and pose graph optimization at the same time. (If you cannot run it in real time, it's ok. Because your computer serves as both multiple robots and ground station.)

**3.1 Euroc dataset**

3.1.1 Download [EuRoC MAV Dataset](http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Although it contains stereo cameras, we only use one camera. We run multiple sequences simultaneously and merge them in one global frame.

3.1.2 Modify the dataset path in the launch file (vins_estimator/launch/euroc_multi_agent.launch)
```
<arg name="sequence_1" default = "YOUR_PATH/MH_01_easy/MH_01_easy.bag" />
<arg name="sequence_2" default = "YOUR_PATH/MH_02_easy/MH_02_easy.bag" />
<arg name="sequence_3" default = "YOUR_PATH/MH_03_medium/MH_03_medium.bag" />
```
3.1.3 Open terminal
```
roslaunch vins_estimator euroc_multi_agent.launch
```

![euroc result](https://github.com/qintonguav/Co-VINS/blob/vins_swarm/support_files/image/euroc_result.png "euroc result")

**3.2 Our data (video)**

Reproduction of part II in video.

3.2.1 Download [Our Data](https://www.dropbox.com/sh/o20itggdn3bgiow/AABxHkz4aWPnRAHzTuORXQnCa?dl=0). 

3.2.2 Modify the dataset path in the launch file (vins_estimator/launch/A3_swarm.launch)
```
<arg name="sequence_1" default = "YOUR_PATH/1.bag" />
<arg name="sequence_2" default = "YOUR_PATH/2.bag" />
<arg name="sequence_3" default = "YOUR_PATH/3.bag" />
<arg name="sequence_4" default = "YOUR_PATH/4.bag" />
```
3.2.3 Open terminal
```
roslaunch vins_estimator A3_swarm.launch
```
(If it gose well, you can reproduce the part II in video. )
![swarm result](https://github.com/qintonguav/Co-VINS/blob/vins_swarm/support_files/image/swarm_result.png "swarm result")

## 4. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, and a generic [camera model](https://github.com/hengli/camodocal).

## 5. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong QIN <tong.qinATconnect.ust.hk>.

For commercial inquiries, please contact Shaojie SHEN <eeshaojieATust.hk>
