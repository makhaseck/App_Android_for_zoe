#!/bin/bash

code .

# launch ROSbridge in the first tab
gnome-terminal --tab -e "roslaunch rosbridge_server rosbridge_websocket.launch"

# launch turtlesim_node in the second tab
gnome-terminal --tab -e "rosrun turtlesim turtlesim_node"

# launch React Native project in the third tab
gnome-terminal --tab -e "npm start"


