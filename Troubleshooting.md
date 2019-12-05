This is the troubleshooting guide to clone this clone this repository in your local device

1) error: could not find a package configuration file provided by "grid_map_core": 
   solution: sudo apt-get install ros-melodic-grid-map

2) error: Could not find a package configuration file provided by "darknet_ros_msgs"
   solution:  i)remove "anahita_msgs","darknet_ros_msgs" - master_layer/CMakeLists.txt (line no's-91)
              ii)remove "darknet_ros_msgs" - master_layer/CMakeLists.txt (line no's-19)
             iii)master_layer/src/local_vision_odom.cpp: comment line no's- 10,11,38-63,195
             iv)master_layer/package.xml: remove line 62,80
             
3)error: Could not find a package configuration file provided by "usb_cam
  solution: sudo apt-get install ros-melodic-usb-cam
  
5)remove libfovis from navigation layer   