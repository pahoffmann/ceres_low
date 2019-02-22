HowTo

1) Realsense anschließen
2) roslaunch realsense2_camera rs_camera.launch filters:=pointcloud
3) catkin_make
4) rosrun my_pcl_tutorial fitlerpipeline input:=/camera/depth/color/points
