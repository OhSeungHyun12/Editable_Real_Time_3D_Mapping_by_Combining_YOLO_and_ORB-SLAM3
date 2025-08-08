cmake_minimum_required(VERSION 3.10)
project(yolo_orb3_ros2)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Check the ORB-SLAM3 path environment variable
if(NOT DEFINED ENV{ORB_SLAM3_ROOT})
  message(FATAL_ERROR "Environment variable ORB_SLAM3_ROOT is not set. Check ~/.bashrc")
endif()
set(ORB_SLAM3_ROOT $ENV{ORB_SLAM3_ROOT})
include_directories(${ORB_SLAM3_ROOT} ${ORB_SLAM3_ROOT}/include)

# Finding ROS 2 and Dependent Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED) 
find_package(Eigen3 3 REQUIRED) 
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Setting and finding the Torch (libtorch) path
set(Torch_DIR "${ORB_SLAM3_ROOT}/Thirdparty/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${ORB_SLAM3_ROOT}/Thirdparty/libtorch/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# ==============================================================================
# Creating target for the ORB_SLAM3 library
# ==============================================================================
add_library(ORB_SLAM3::ORB_SLAM3 SHARED IMPORTED)
set_target_properties(ORB_SLAM3::ORB_SLAM3 PROPERTIES
  IMPORTED_LOCATION "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so"
  INTERFACE_INCLUDE_DIRECTORIES "${ORB_SLAM3_ROOT}/include;${ORB_SLAM3_ROOT}/include/CameraModels"
)
add_library(ORB_SLAM3::g2o SHARED IMPORTED)
set_target_properties(ORB_SLAM3::g2o PROPERTIES
  IMPORTED_LOCATION "${ORB_SLAM3_ROOT}/Thirdparty/g2o/lib/libg2o.so"
)
add_library(ORB_SLAM3::DBoW2 SHARED IMPORTED)
set_target_properties(ORB_SLAM3::DBoW2 PROPERTIES
  IMPORTED_LOCATION "${ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib/libDBoW2.so"
)

# include
include_directories(
  ${Pangolin_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
  "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
)

# ==============================================================================
# Build and link libraries.
# ==============================================================================
add_executable(mono src/ros_mono.cc)
ament_target_dependencies(mono rclcpp cv_bridge sensor_msgs image_transport tf2 tf2_ros)
target_link_libraries(mono
    ORB_SLAM3::ORB_SLAM3
    ORB_SLAM3::g2o
    ORB_SLAM3::DBoW2
    ${OpenCV_LIBS}
    ${TORCH_LIBRARIES}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    Boost::system
)

add_executable(rgbd src/ros_rgbd.cc)
ament_target_dependencies(rgbd rclcpp cv_bridge sensor_msgs image_transport tf2 tf2_ros)
target_link_libraries(rgbd
    ORB_SLAM3::ORB_SLAM3 ORB_SLAM3::g2o ORB_SLAM3::DBoW2
    ${OpenCV_LIBS} ${TORCH_LIBRARIES} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES} Boost::system
)

add_executable(mono_ar src/AR/ros_mono_ar.cc src/AR/ViewerAR.cc)
ament_target_dependencies(mono_ar rclcpp cv_bridge sensor_msgs image_transport tf2 tf2_ros)
target_link_libraries(mono_ar
    ORB_SLAM3::ORB_SLAM3 ORB_SLAM3::g2o ORB_SLAM3::DBoW2
    ${OpenCV_LIBS} ${TORCH_LIBRARIES} ${EIGEN3_LIBS} ${Pangolin_LIBRARIES} Boost::system
)

# Installing executables and launch files
install(TARGETS mono rgbd mono_ar DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
