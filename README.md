# Real_Time_3D_map_generation_with_a_combination_of_YOLO_and_ORB_SLAM
**This project combines YOLO for object detection and ORB-SLAM for real-time SLAM to generate 3D maps.**

this project uses YOLOv11 for object detection and integrates the detected object into a 3D map built with ORB-SLAM3. The goal is to generate a high-precision 3D map that accurately represents the position, shape, and surrounding structure of the detedcted objects.

## Keyword
+  **Object Detection**

    Real-time object detection using YOLOv11.
+  Dynamic SLAM
  
    ORB-SLAM3 provides real-time camera tracking and 3D map reconstruction in dynamic environments.
+  3D Map Generation
  
    Generate high-precision 3D maps that visually represent detected objects and the surrounding terrain based on object recognition.
---

## Getting Started

### 1. Prerequisites

> **OS:**  Ubuntu 24.04

> **OpenCV:** 4. 6. 0

> **Eigen3:** 3.4.0

> **Pangolin:** 0.9.3

> **ROS:** jazzy

### 2. Installation

#### ROS2 Jazzy

> **Set Locale**
```
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

> **Enable required repositories**
```
sudo apt install software-properties-common
sudo add-apt-repository universe
```

> **Enable required repositories**
```
$ sudo apt update && sudo apt install curl -y
$ export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
$ curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
$ sudo dpkg -i /tmp/ros2-apt-source.deb
```

> **Install development tools (optional)**
```
sudo apt update && sudo apt install ros-dev-tools
```

> **Install ROS 2**
```
sudo apt update && sudo apt upgrade
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool
```

> **Make venv**
```
sudo apt install python3 python3-venv python3-pip
python3 -m venv ~/venvs/ORB_SLAM3_venv
```

> **Setup environment ROS 2**
```
code ~/.bashrc
```
You can add to this code or change it if you want.
```
echo -e "alias list:\n\r jazzy"
alias ros_domain="export ROS_DOMAIN_ID=13; echo \"ROS_DOMAIN_ID=13\""
alias jazzy="source /opt/ros/jazzy/setup.bash; ros_domain; echo \"ROS2 jazzy is activated!\""

echo "To see env list type: envs"
alias envs='ls ~/envs'
alias orbslam3_env='source ~/envs/orbslam3_env/bin/activate'

```

> **Test**
```
ros2 --help
ros2 topic list
printenv ROS_DISTRO  # Check the output: "jazzy" 
```

#### OpenCV

> **Install OpenCV**
```
sudo apt install libopencv-dev python3-opencv
```

> **Test**
```
pkg-config --modversion opencv4
```

#### Eigen3

> **Install Eigen3**
```
sudo apt install libeigen3-dev
```

> **Test**
```
pkg-config --modversion eigen3
```

#### Pangolin 

> **Install Pangolin**
```
sudo apt update
sudo apt install libepoxy-dev
pip install wheel
mkdir YOLO_ORB_SLAM3
cd ~/YOLO_ORB_SLAM3/Pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

#### ORB-SLAM

> **Install ORB-SLAM**
```
sudo apt update
sudo apt install cmake build-essential git libgtk-3-dev
sudo apt install libeigen3-dev libboost-all-dev libglew-dev
sudo apt install libtbb-dev libx11-dev libqt5opengl5-dev qtbase5-dev
cd ~/YOLO_ORB_SLAM3
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ~/YOLO_ORB_SLAM3/ORB_SLAM3
chmod +x build.sh
```

#### Intel RealSense cammera

> **Install RealSense driver**
```
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

> Replace with modified file that overwrites specific files inside ORB SlAM

### 3. Build

#### Build ORB-SLAM3

> **Build**
```
cd ~/YOLO_ORB_SLAM3/ORB_SLAM3
./build.sh
```

