cmake_minimum_required(VERSION 3.5)
project(KinectCapture)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} $ENV{HOME}/freenect2/include)

add_executable(kinect_capture kinect_capture.cpp)
target_link_libraries(kinect_capture ${OpenCV_LIBS} $ENV{HOME}/freenect2/lib/libfreenect2.so)

