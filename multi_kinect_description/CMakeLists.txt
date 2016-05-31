cmake_minimum_required(VERSION 2.8.3)
project(multi_kinect_description)
find_package(catkin REQUIRED)

find_package(roslaunch)

roslaunch_add_file_check(test/multi_kinectv2_rviz_launch_test.xml)