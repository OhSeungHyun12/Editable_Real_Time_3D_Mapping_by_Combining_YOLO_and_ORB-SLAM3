cmake_minimum_required(VERSION 3.10)
project(YOLO_ORB_SLAM3)   # ROS2 패키지명은 소문자가 권장됨

# C++17 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ROS 2 빌드
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# OpenCV
find_package(OpenCV 4.4 REQUIRED)
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# Eigen, Pangolin
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# Boost
find_package(Boost REQUIRED COMPONENTS system)

# Torch (libtorch)
set(Torch_DIR "${PROJECT_SOURCE_DIR}/Thirdparty/libtorch/share/cmake/Torch")
find_package(Torch REQUIRED)

# 컴파일 옵션
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# include 경로
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/CameraModels
  ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${TORCH_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/Thirdparty/libtorch/include
  ${PROJECT_SOURCE_DIR}/Thirdparty/libtorch/include/torch/csrc/api/include
)

# 라이브러리 목록
set(LIBS 
  ${OpenCV_LIBS}
  ${TORCH_LIBRARIES}
  ${Pangolin_LIBRARIES}
  Boost::system
  ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# 실행 파일 생성
add_executable(mono src/ros_mono.cc)
target_link_libraries(mono ${LIBS})
ament_target_dependencies(mono rclcpp)

# 설치 설정
install(TARGETS mono
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
