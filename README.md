# Software Integration

"XicroExample" is an example firmware interface between Xicro and Arduino Rev2.   
"scripts" is an example software_integration package ..............   

## Installation
1.) Clone the repo. You must unzip and put each folder in the src directory of your workspace.

2.) Build "software_integration_solution" in your workspace.
```
cd ~/[your_workspace]
colcon build
source install/setup.bash
```
3.) Install python lib
```
pip3 install paho-mqtt
```
## Testing out Tele-operation System

# Master Side

Terminal 1: Open Xicro Node
```
ros2 run xicro_pkg xicro_node_demo_ID_1_arduino.py 
```
Terminal 2: Turtlesim_Plus
```
ros2 run software_integration_solution mqtt_xicro.py
```

# Slave Side

Terminal 1: Turtlesim_Plus
```
ros2 run turtlesim_plus turtlesim_plus_node.py
```
Terminal 2: Master Node
```
ros2 run software_integration_solution mqtt_turtlesim.py
```
