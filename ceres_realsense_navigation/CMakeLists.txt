cmake_minimum_required(VERSION 2.8.3)
project(ceres_navigation)



find_package(catkin REQUIRED   
    std_msgs
    rospy
    message_generation)

add_service_files(
  FILES
  SetController.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package()

