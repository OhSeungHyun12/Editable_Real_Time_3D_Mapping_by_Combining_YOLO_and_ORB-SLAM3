## Project in progress...

# Real_Time_3D_map_generation_with_a_combination_of_YOLO_and_ORB_SLAM
**This project combines YOLO for object detection and ORB-SLAM for real-time SLAM to generate 3D maps.**

this project uses YOLOv11 for object detection and integrates the detected object into a 3D map built with ORB-SLAM3. The goal is to generate a high-precision 3D map that accurately represents the position, shape, and surrounding structure of the detected objects.

<img width="1377" height="771" alt="image" src="https://github.com/user-attachments/assets/0ca9e2bc-ba48-4d3f-93c7-52494b500973" />


## Keyword
+  **Object Detection**

    Real-time object detection using YOLOv11.
+  Dynamic SLAM
  
    ORB-SLAM3 provides real-time camera tracking and 3D map reconstruction in dynamic environments.
+  3D Map Generation
  
    Generate high-precision 3D maps that visually represent detected objects and the surrounding terrain based on object recognition.
---

## License

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
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

> **Enable required repositories**
```
sudo apt install software-properties-common -y
sudo add-apt-repository universe
```

> **Install ROS 2 APT Source**
```
sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

# Install ROS 2 and dev tools
```
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions python3-rosdep python3-vcstool ros-dev-tools \
                    ros-jazzy-topic-tools ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-v4l2-camera
```

> **Make Virtual Environment**
```
sudo apt install python3 python3-venv python3-pip -y
python3 -m venv ~/venvs/ORB_SLAM3_venv
```

> **Setup environment ROS 2**
```
code ~/.bashrc
```
You can add to this code or change it if you want.
```
# ---------------------------
# Aliases list
# ---------------------------
echo "To see aliases list type: my_aliases"
my_aliases() {
    echo -e "💡 List of available aliases."
    echo "------------------------------------"
    echo "▸ ros:"
    echo "  - jazzy             : Activate ROS2 Jazzy ."
    echo "  - jazzy_yolo_orb3   : Set Jazzy workspacem, ORB_SLAM3 package path."
    echo "▸ venvs               : Virtual Environment List."
    echo "------------------------------------"
}

# ---------------------------
# ROS 2 Jazzy Aliases
# ---------------------------
alias ros_domain="export ROS_DOMAIN_ID=13; echo \"ROS_DOMAIN_ID=13\""
alias jazzy="source /opt/ros/jazzy/setup.bash; ros_domain; echo \"ROS2 jazzy is activated!\""

# ---------------------------
# Virtual Environment
# ---------------------------
#
alias venvs="ls ~/venvs"
alias ORB_SLAM3_venv="source ~/venvs/ORB_SLAM3_venv/bin/activate"
alias yolo11="source ~/venvs/yolo11/bin/activate"

# ---------------------------
# ROS Package Path
# ---------------------------
# 
# ---------------------------
# ROS Package Path + ORB-SLAM3 env
# ---------------------------
jazzy_yolo_orb3() {
    source /opt/ros/jazzy/setup.bash && ros_domain

    # ros2_ws overlay
    local ws_setup="$HOME/ros2_ws/install/setup.bash"
    if [ -f "$ws_setup" ]; then
        source "$ws_setup"
    fi

    # ORB-SLAM3 ROOT
    export ORB_SLAM3_ROOT="$HOME/YOLO_ORB_SLAM3/ORB_SLAM3"
    if [ ! -d "$ORB_SLAM3_ROOT" ]; then
        echo "⚠️ ORB_SLAM3_ROOT dir not found: $ORB_SLAM3_ROOT"
    fi

    # Add runtime library path (ORB-SLAM3, libtorch)
    case ":$LD_LIBRARY_PATH:" in
        *":$ORB_SLAM3_ROOT/lib:"*) ;;
        *) export LD_LIBRARY_PATH="$ORB_SLAM3_ROOT/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}";;
    esac
    case ":$LD_LIBRARY_PATH:" in
        *":$ORB_SLAM3_ROOT/Thirdparty/libtorch/lib:"*) ;;
        *) export LD_LIBRARY_PATH="$ORB_SLAM3_ROOT/Thirdparty/libtorch/lib:$LD_LIBRARY_PATH";;
    esac

    # Package source path(yolo_orb3_ros2)
    local pkg_dir="$HOME/ros2_ws/src/yolo_orb3_ros2"
    if [ -d "$pkg_dir" ]; then
        case ":$ROS_PACKAGE_PATH:" in
            *":$pkg_dir:"*) ;;
            *)
                export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:+$ROS_PACKAGE_PATH:}$pkg_dir"
                echo "ROS_PACKAGE_PATH += $pkg_dir"
                ;;
        esac
    fi

    echo "Jazzy + workspace overlay + ORB_SLAM3 env ready ✅"
    echo "ORB_SLAM3_ROOT=$ORB_SLAM3_ROOT"
}
```

> **Test**
```
ros2 --help
ros2 topic list
printenv ROS_DISTRO  # Check the output: "jazzy" 
```

#### Install OpenCV

```
sudo apt install libopencv-dev python3-opencv -y
pkg-config --modversion opencv4
```

#### Install Eigen3

```
sudo apt install libeigen3-dev -y
pkg-config --modversion eigen3
```

#### Install Pangolin 

```
sudo apt install libepoxy-dev -y
pip install --upgrade wheel setuptools pyyaml

mkdir YOLO_ORB_SLAM3 && cd YOLO_ORB_SLAM3
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

#### Install ORB-SLAM

```
sudo apt install cmake build-essential git libgtk-3-dev libboost-all-dev libglew-dev \
libtbb-dev libx11-dev libqt5opengl5-dev qtbase5-dev -y

cd ~/YOLO_ORB_SLAM3
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
```
Replace with modified file that overwrites specific files inside ORB SlAM

#### Intel RealSense Camera
Since librealsense only supports up to Ubuntu 22, you need to build and install the official source code directly.

> **Installing dependent packages**
```
sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev libudev-dev libopencv-dev -y
```

> **Clone librealsense and build**
```
sudo apt update
cd ~sudo apt install nvidia-cuda-toolkit --fix-missing
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense && mkdir build && cd build

cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true \
         -DBUILD_GRAPHICAL_EXAMPLES=true -DFORCE_LIBUVC=true \
         -DBUILD_WITH_CUDA=false

make -j$(nproc)
sudo make install
```

> **Test camera**
```
realsense-viewer
```

#### YOLOv11

> **Installing LibTorch**

Install CUDA 11.8 LibTorch and put it in ORB_SLAM3/Thirdparty.
```
wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.7.1%2Bcu118.zip
```

### 3. Build

#### Build ORB-SLAM3
```
cd ~/YOLO_ORB_SLAM3/ORB_SLAM3
chmod +x build.sh
./build.sh 2>&1 | tee build.log
```
#### Build ROS2
```
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select yolo_orb3_ros2
source install/setup.bash
```

### 4. Running ORB-SLAM3 

> **Run ROS2**

need two terminal
```
ros2 run v4l2_camera v4l2_camera_node
```

> **Intel RealSense Camera D455**
```
./cam/mono_realsense_D455 Vocabulary/ORBvoc.txt ./cam/mono_RealSense_D455.yaml
```

> **logi**
```
ros2 run yolo_orb3_ros2 mono_ar ~/YOLO_ORB_SLAM3/ORB_SLAM3/Vocabulary/ORBvoc.txt ~/YOLO_ORB_SLAM3/ORB_SLAM3/Examples/ROS2/webcam.yaml
```
### 5. Running YOLO + ORB-SLAM3

> **run Rviz2**
>
```
jazzy_yolo_orb3
ros2 launch yolo_orb3_ros2 yolo_orb3_rviz2.launch.py
```
