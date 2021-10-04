# ROS Network Analysis Package
This is a ROS package that provide tools to analyze wireless network through four key performance metrics: 1) signal quality (RSSI, LQI); 2) application-level network latency 3) throughput (TCP + UDP), 4) connection error metrics (retransmits), between two ROS nodes/computers/machines.

There are four ROS nodes in this package - one for each network performance metric stated above.

#Installation
To use these packages you will have to install this package into your ROS workspace. Make sure you install ROS Melodic and set up your ROS workspace as per the instructions at http://wiki.ros.org/melodic/Installation.  Below are the commands which will help you do that, considering you already have a catkin workspace.
```
  cd ~/catkin_ws
  cd src
  git clone https://github.com/herolab-uga/ros-network-analysis.git
  cd ../..
  catkin_make
  catkin_make install
```
#Usage Instructions

## Network Delay 
Apart from network delay node, other two packages need two ROS nodes, One as listener and another as speaker. There are four packages linkUtiliaztion, NetworkErrors, wireless_quality and network_delay. Below are the commands to use them individually.

## Network throughput

## Network Quality

## Network Errors

explain the node
explain what happens in the node including messages (and custom message format)
```
  rosrun network_analysis node_name
```

linkUtilization.py
This node study the throughput of the interfaces being used. Change the interface name in linkutilization.py file according to the interface you are using (by default it will be wlan0). This needs two ROS nodes : one for transmitting and one for receiving data. Based on the transfer rate you will get the throughput. This will provide the throughput in mbps along with the bytes and data packets transfered between the two nodes.

## Running all nodes together

```
  roslaunch network_analysis
```
