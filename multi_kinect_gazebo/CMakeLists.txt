cmake_minimum_required(VERSION 2.8.3)
project(multi_kinect_gazebo)
find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)
roslaunch_add_file_check(test/multi_kinectv2_gazebo_launch_test.xml)