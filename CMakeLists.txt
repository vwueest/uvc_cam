cmake_minimum_required(VERSION 2.8.3)
project(uvc_cam)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager_py
  cv_bridge
  image_proc
  rospy
  sensor_msgs
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/cam_params.cfg
)
#add_dependencies(uvc_cam_node ${PROJECT_NAME}_gencfg)

catkin_package()

