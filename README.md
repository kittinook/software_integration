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
# Testing out Tele-operation System

## Simple MQTT Example
## Master-Slave Tele-operation with Xicro Interface

# Simple MQTT Example

## Master Computer

Terminal 1: Open MQTTMaster Node
```
ros2 run software_integration mqtt_master.py 
```
Terminal 2: Open Command Line
```
ros2 topic pub /pub2mqtt std_msgs/msg/String "data: 'Hello FIBO !!!

## Master Computer

Terminal 1: Open MQTTSlave Node
```
ros2 run software_integration mqtt_slave.py 
```
Terminal 2: Open Command Line
```
ros2 topic echo /mqtt2sub


# Master-Slave Tele-operation with Xicro Interface

## Master Computer

Terminal 1: Open Xicro Node
```
ros2 run xicro_pkg xicro_node_demo_ID_1_arduino.py 
```
Terminal 2: Turtlesim_Plus
```
ros2 run software_integration mqtt_xicro.py
```

## Slave Computer

Terminal 1: Turtlesim_Plus
```
ros2 run turtlesim_plus turtlesim_plus_node.py
```
Terminal 2: Master Node
```
ros2 run software_integration mqtt_turtlesim.py
```
