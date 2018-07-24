**Multipup Drone Control and Latency Analysis**

This repository contains necessary files for controlling more than one drone connected in the same network. The drones can also be made to perform coordinated motion where each drone follows a leader in the network. Several coordinated tasks have been presented and the results are verified by simulations on Gazebo, where certain amount of communication delay between the drones is also considered. Pluto drones are mounted with esp8266 Wi-Fi module for communication.
ESP wifi module set to both Station and Access point modes.\
Accessing ESP using telnet:-\

telnet IP Address\
Set Station SSID and pssword to connect:-\
+++AT STA SSID PASS\
\
Set Mode:-\
+++AT MODE 3\
\
Password of different Nodes are:-\
Pluto_NODE-0  Node___0\
Pluto_NODE-1  Node___1\
Pluto_NODE-2  Node___2\
\
Multiple nodes to communicate, localise and control the drones. For each drone in the network, a unique topic is defined. Drone commands are published on this topic while each drone subscribes to it and sets its parameters accordingly. For localization, whycon markers are mounted on each drone. Using socket programming, each drone is connected to the server. The drones reach out to the server for commands. Drone commands are published on topic 'drone_command_x' while the sensor values are published on topics 'Sensor_data_x'.
