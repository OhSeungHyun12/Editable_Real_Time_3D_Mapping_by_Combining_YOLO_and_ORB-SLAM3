cmake_minimum_required(VERSION 3.10)
project(yolo_orb3_ros2)

# C++17 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ORB-SLAM3 경로 설정
if(NOT DEFINED ENV{ORB_SLAM3_ROOT})
  message(FATAL_ERROR "환경변수 ORB_SLAM3_ROOT가 설정되지 않았습니다.")
endif()

set(ORB_SLAM3_ROOT $ENV{ORB_SLAM3_ROOT})

# ROS 2 빌드
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)

# OpenCV
find_package(OpenCV 4.4 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# Eigen, Pangolin
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS system)

# Torch (libtorch)
set(Torch_DIR "${ORB_SLAM3_ROOT}/Thirdparty/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${ORB_SLAM3_ROOT}/Thirdparty/libtorch/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# 컴파일 옵션
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# include 경로
include_directories(
  ${ORB_SLAM3_ROOT}
  ${ORB_SLAM3_ROOT}/include
  ${ORB_SLAM3_ROOT}/include/CameraModels
  ${ORB_SLAM3_ROOT}/Thirdparty/Sophus
  ${Pangolin_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
)

# 라이브러리 목록
set(LIBS 
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so
  ${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so
  Boost::system
)

# Mono 카메라 노드
add_executable(mono src/ros_mono.cc)
target_link_libraries(mono ${LIBS})
ament_target_dependencies(mono rclcpp cv_bridge sensor_msgs image_transport tf2 tf2_ros)

# RGB-D 카메라 노드
add_executable(rgbd src/ros_rgbd.cc)
target_link_libraries(rgbd ${LIBS})
ament_target_dependencies(rgbd rclcpp cv_bridge sensor_msgs image_transport tf2 tf2_ros)

# Mono-AR 카메라 노드
add_executable(mono_ar
src/AR/ros_mono_ar.cc
src/AR/ViewerAR.cc )
target_link_libraries(mono_ar ${LIBS})
ament_target_dependencies(mono_ar rclcpp cv_bridge sensor_msgs image_transport tf2 tf2_ros)

# # Stereo 카메라 노드
# add_executable(stereo src/ros_stereo.cc)
# target_link_libraries(stereo ${LIBS})
# ament_target_dependencies(stereo rclcpp)

# # Monocular-Inertial 카메라 노드
# add_executable(mono_inertial src/ros_mono_inertial.cc)
# target_link_libraries(mono_inertial ${LIBS})
# ament_target_dependencies(mono_inertial rclcpp)

# # Stereo-Inertial 카메라 노드
# add_executable(stereo_inertial src/ros_stereo_inertial.cc)
# target_link_libraries(stereo_inertial ${LIBS})
# ament_target_dependencies(stereo_inertial rclcpp)

# 실행 파일 설치
install(TARGETS
  mono
  rgbd
  mono_ar
  # stereo
  # mono_inertial
  # stereo_inertial
  DESTINATION lib/${PROJECT_NAME})

# launch 파일 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
