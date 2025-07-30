cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM3)

# Check C++17 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ROS 2 빌드
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# OpenCV
find_package(OpenCV 4.4 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Torch REQUIRED)


# 컴파일 옵션
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# 라이브러리 경로
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)
set(CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/../../../Thirdparty/libtorch)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${PROJECT_SOURCE_DIR}/../../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../../Thirdparty/Sophus
${Pangolin_INCLUDE_DIRS}
${TORCH_INCLUDE_DIRS}
)

set(LIBS 
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES} 
  ${Pangolin_LIBRARIES}
  Boost::system
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
  ${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM3.so
)

# 실행 파일 생성
add_executable(mono src/ros_mono.cc)
target_link_libraries(mono ${LIBS})
ament_target_dependencies(mono rclcpp)

# add_executable(stereo src/ros_stereo.cc)
# target_link_libraries(stereo ${LIBS})
# ament_target_dependencies(stereo rclcpp)

# add_executable(rgbd src/ros_rgbd.cc)
# target_link_libraries(rgbd ${LIBS})
# ament_target_dependencies(rgbd rclcpp)

# add_executable(mono_inertial src/ros_mono_inertial.cc)
# target_link_libraries(mono_inertial ${LIBS})
# ament_target_dependencies(mono_inertial rclcpp)

# add_executable(stereo_inertial src/ros_stereo_inertial.cc)
# target_link_libraries(stereo_inertial ${LIBS})
# ament_target_dependencies(stereo_inertial rclcpp)

# 패키지 선언
ament_package()
