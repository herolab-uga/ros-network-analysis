# ROS Network Analysis Package
This is a ROS package that provide tools to analyze wireless network through four key performance metrics: 1) signal quality (RSSI, LQI); 2) application-level network latency 3) throughput (TCP + UDP), 4) connection error metrics (retransmits), between two ROS nodes/computers/machines.

There are five ROS nodes in this package - two ROS nodes are for measuring network delay (Server and client side nodes), and three ROS nodes are for measuring quality, throughput, and errors. Each metric has a custom defined ROS message type as discussed below.

# Installation
To use these packages you will have to install this package into your ROS workspace. Make sure you install ROS Melodic and set up your ROS workspace as per the instructions at http://wiki.ros.org/melodic/Installation.  Below are the commands which will help you do that, considering you already have a catkin workspace.
```
  cd ~/catkin_ws
  cd src
  git clone https://github.com/herolab-uga/ros-network-analysis.git
  cd ../..
  catkin_make
  catkin_make install
```
# Usage Instructions

## Network Delay 
This _network_delay_ ROS node records the delay in milliseconds (ms). It records the application-level network rount trip time latency, it also records if the network is alive (connected) or not. If the network is dead or temporarily unavailable the delay value will be -1 ms. Use below command to run this node.

on server side
```
  rosrun network_analysis pingactionserver
```
on client side

```
  rosrun network_analysis network_delay
```
This node pings the client everysecond and wait for 2 seconds before timeout. If the network is alive the network status will be 1 otherwise 0.

### ROS Topics
The node publishes measurements in the ROS topic: "/network_analysis/network_delay" by default

### ROS Message type
This ROS node uses the below custom message type (msgs/NetworkDelay.msg) when it publishes information.

_string iface_ #name of the wireless interface (e.g. wlan0, wlan1, etc.)

_float32 delay_ #network delay obtained using the ros service call (application level roung trip time without using ICMP echo request)

_bool alive_ #Flag to check if the link is alive or dead

### ROS Parameters
There are two ROS parameters associated with this node. 

"update_rate_network_delay" -> sets the message publishing frequency

"timeout_network_delay" --> sets the timeout in ms when the node should read a connection loss.

## Network Throughput
This _link_utilization_ ROS node records the throughput of the network in Mbps. It records the total transmitted and recieved data packets and data rates in Mbps from tcp and udp. It uses /proc/net/dev file to look for the interface name and fetch the required data from /proc/net/snmp file for each tcp and udp connection. It will print the message if the interface name does not exist or disconnected. You can use the below command to run this node.

```
  rosrun network_analysis link_utilization.py
```
This command will print "Total throughput on interface $interface_name is Transmit x Mbps and Receive y Mbps" every second.

### ROS Topics
The node publishes measurements in the ROS topic: "/network_analysis/link_utilization" by default


### ROS Message type
The delay node uses the below custom message type (msgs/LinkUtilitzation.msg) when publishing information.

_string iface_ #name of the wireless interface (e.g. wlan0, wlan1, etc.)

#TCP related information on link utilization for a given (NIC) interface

_int64 tcp_tx_segments_

_int64 tcp_rx_segments

float64 tcp_tx_segmentrate

float64 tcp_rx_segmentrate_

#UDP related information on link utilization for a given (NIC) interface

_int64 udp_tx_datagrams

int64 udp_rx_datagrams

float64 udp_tx_datagramrate

float64 udp_rx_datagramrate_

#Total (IP: TCP + UDP) link utilization for a given (NIC) interface

_int64 total_tx_packets

int64 total_tx_bytes

int64 total_rx_packets

int64 total_rx_bytes

float64 total_tx_mbps

float64 total_rx_mbps_

### ROS Parameters
There are two ROS parameters associated with this node. 
"INTERFACE_NAME" --> sets the device id of the network interface. By default it is set to "wlan0"
"update_rate_network_delay" -> sets the message publishing frequency


## Network Quality
This node records the RSSI value of the network in dBm. 

```
  rosrun network_analysis wireless_quality.py
```


## Network Errors
This node records the network error metrics.

```
  rosrun network_analysis network_errors.py
```



## Running all nodes together

Use the provided ROS launch files (in XML format). 
he "client.launch" should be run at the client side where the performance is measured. For instance, this would be run on a mobile robot. 
The "server.launch" should be run at a remote station to which the client is communitating its data to. For instance, this would be a command station computer from where a robot is controlled.

on server side
```
  roslaunch network_analysis server.launch
```
on client side

```
  roslaunch network_analysis client.launch
```



## Core contributors

* **Pranav Pandey** - PhD student

* **Dr.Ramviyas Parasuraman** - Principal Investigator


## Heterogeneous Robotics (HeRoLab)

This project is a part of a Learning Technology Grant (LTG) project at the Heterogeneous Robotics Research Lab (HeRoLab) of the University of Georgia.

Please contact hero at uga . edu for any queries

http://hero.uga.edu/

<p align="center">
<img src="http://hero.uga.edu/wp-content/uploads/2021/04/herolab_newlogo_whitebg.png" width="300">
</p>


