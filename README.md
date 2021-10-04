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
This node records the delay in milliseconds (ms). It uses interface name as a parameter and records the delay, it also records if the network is alive or not. If the network is dead or temporarily unavailable the delay will be -1. Use below command to run this node .

# on server side
```
  rosrun network_analysis pingactionserver
```
# on client side

```
  rosrun network_analysis network_delay
```
This node pings the client everysecond and wait for 2 seconds before timeout. If the network is alive the network status will be 1 otherwise 0.

## Network throughput
This node records the throughput of the network in Mbps. It uses the interface name as a parameter ( by default it is wlan0) and records the total transmitted and recieved data packets and bytes from tcp and udp. It uses /proc/net/dev file to look for the interface name and fetch the required data from /proc/net/snmp file for each tcp and udp connection. It will print the message if the interface name does not exist or disconnected. You can use the below command to run this node.

```
  rosrun network_analysis link_utilization.py
```
This command will print "Total throughput on interface $interface_name is Transmit x Mbps and Receive y Mbps" everysecond.

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
