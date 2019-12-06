# Troubleshooting

This is the troubleshooting guide to clone this clone this repository in your local device.

1. Could not find a package configuration file provided by "darknet_ros_msgs"
   
   Solution:  
   i) remove "anahita_msgs","darknet_ros_msgs" - master_layer/CMakeLists.txt (line no's-91)
   ii)  remove "darknet_ros_msgs" - master_layer/CMakeLists.txt (line no's-19)
   iii) master_layer/src/local_vision_odom.cpp: comment line no's- 10,11,38-63,195
   iv)  master_layer/package.xml: remove line 62,80

2. Remove libfovis from navigation layer