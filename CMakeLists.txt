cmake_minimum_required(VERSION 3.0.2)
project(bitirme_projesi)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
)

catkin_package()
