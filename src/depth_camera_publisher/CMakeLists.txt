cmake_minimum_required(VERSION 3.0.2)
project(depth_camera_publisher)
find_package(catkin REQUIRED COMPONENTS
  rospy
)
catkin_python_setup()
catkin_package(CATKIN_DEPENDS rospy)
