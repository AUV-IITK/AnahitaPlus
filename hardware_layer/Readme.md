# Instructions, connections and configuration for some hardware products

This repository contains the code for our second underwater vehicle `Anahita`. 

1. **Waterlinked A50 DVL** is connected to PC via ethernet and is powered via battery because its voltage rating 10-30 V . Configure the ip address of your connection as 192.168.194.90 on same subnet as DVl ,i.e, 192.168.194.95 and then can be run via intrustions in the dvl-a50 ros package. To configure the ip for DVL use the command **sudo ip addr add 192.168.194.90/24 dev eth1** dont configure from network settings. And dont forget to run roscore service before running publisher.py.


```
