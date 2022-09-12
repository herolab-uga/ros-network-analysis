# ROS Network Analysis Package
This is a ROS package that provide tools to analyze wireless network through four key performance metrics: 1) signal quality (RSSI, LQI); 2) application-level network latency 3) throughput (TCP + UDP), 4) connection error metrics (retransmits), between two ROS nodes/computers/machines.

There are five ROS nodes in this package - two ROS nodes are for measuring network delay (Server and client side nodes), and three ROS nodes are for measuring quality, throughput, and errors. Each metric has a custom defined ROS message type as discussed below.

# Publication
If you use this work, please cite our paper:
P. Pandey and R. Parasuraman, "Empirical Analysis of Bi-directional Wi-Fi Network Performance on Mobile Robots in Indoor Environments," 2022 IEEE 95th Vehicular Technology Conference: (VTC2022-Spring), 2022, pp. 1-7, doi: 10.1109/VTC2022-Spring54318.2022.9860438.

https://ieeexplore.ieee.org/abstract/document/9860438 


# Dependencies

Dependencies: netstat, ethtool 

(sudo apt-get install net-tools ethtool)

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
This node pings the client everysecond and wait for 2 seconds before timeout. If the network is alive the network status will be 1 otherwise 0. Running this node will print "ROS Ping Action finished: SUCCEEDED" if the network is alive otherwise it will print "Action did not finish before the timeout. May be a network problem or may be the ping action server node stopped. See network_delay.alive history for more diagnosis". In the rosbag it will record network delay along with interface name, alive status, timestamp, header_seq and frame_id.

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
The link utilization node uses the below custom message type (msgs/LinkUtilitzation.msg) when publishing information.

_string iface_ #name of the wireless interface (e.g. wlan0, wlan1, etc.)

#TCP related information on link utilization for a given (NIC) interface

_int64 tcp_tx_segments_

_int64 tcp_rx_segments_

_float64 tcp_tx_segmentrate_

_float64 tcp_rx_segmentrate_

#UDP related information on link utilization for a given (NIC) interface

_int64 udp_tx_datagrams_

_int64 udp_rx_datagrams_

_float64 udp_tx_datagramrate_

_float64 udp_rx_datagramrate_

#Total (IP: TCP + UDP) link utilization for a given (NIC) interface

_int64 total_tx_packets_

_int64 total_tx_bytes_

_int64 total_rx_packets_

_int64 total_rx_bytes_

_float64 total_tx_mbps_

_float64 total_rx_mbps_

### ROS Parameters
There are two ROS parameters associated with this node. 

"INTERFACE_NAME" --> sets the device id of the network interface. By default it is set to "wlan0"

"update_rate" -> sets the message publishing frequency


## Network Quality
This node records the RSSI value of the network in dBm. Received Signal Strength Indicator (RSSI) is a measurement of the power present in a received wireless signal. This node reads the /proc/net/wireless for the given interface name to publish or record all the signal quality data.

```
  rosrun network_analysis wireless_quality.py
```

This command will print "Initialized measurement of wireless quality of _iface_ interface" and starts publishing the RSSI values or it will print "The specified interface _iface_ does not exist or is disconnected. Please check" or any other error message if there is any error. 

This node will provide the rssi, lqi and noise data along with the timestamp and other fields mentioned in the ROS message type.

### ROS Topics
The node publishes measurements in the ROS topic: "network_analysis/wireless_quality" by default


### ROS Message type
The delay node uses the below custom message type (msgs/WirelessLink.msg) when publishing information.

#name of the wireless interface (e.g. wlan0, wlan1, etc.)

_string iface_

#ssid of the access point (e.g. ROBOT0, ROBOT1, CommandStation, etc.)

_string ssid_

#Connection status (1 is connected and 0 is disconnected/error0

_bool status_ 

#Received Signal Strength (RSS) in dBm

_int32 txpower_

_int32 rssi_

#Link Quality of the wireless link in percentage (scale of 1 to 100)

_float32 lqi_

#Noise floor of the wireless link in dBm (only limited NICs provide this correctly)

_int32 noise_


### ROS Parameters
There are two ROS parameters associated with this node. 

"INTERFACE_NAME" --> sets the device id of the network interface. By default it is set to "wlan0"

"update_rate" -> sets the message publishing frequency

## Network Errors
This node records the network error metrics. This nodes publish the total numnber of data for each retransmitted, retires, etc. This access the netstat for fetching the retransmitted, bad segments and retires and ethtool for fetching the rx_dropped and tx_retires data. This node reads the file /sys/class/net/" + interfacename +"/statistics/tx_errors (or tx_dropped or rx_errors or rx_dropped) to publish these data.

```
  rosrun network_analysis network_errors.py
```

This command will print "Launched the network_errors node to monitor the retries, drops, errors in the network link" and starts publishing the data or it will print "For ethtool, the specified interface %s does not exist or is disconnected. Reporting only global network errors (not interface specific)." or any other error message if it is not able to publish it.

### ROS Topics
The node publishes measurements in the ROS topic: "network_analysis/network_errors" by default


### ROS Message type
The delay node uses the below custom message type (msgs/NetworkErrors.msg) when publishing information.

_string iface_ #name of the wireless interface (e.g. wlan0, wlan1, etc.)

#segment errors at (tcp) protocol level

_int64 retransmits_

_int64 badsegments_

#errors in udp transmission

_int64 udperrors_

#system level (MAC layer) errors

_int64 tx_retires_

_int64 rx_dropped_


#interface level (NIC statistics) errors

_int64 nic_tx_errors_

_int64 nic_rx_errors_

_int64 nic_tx_dropped_

_int64 nic_rx_dropped_


### ROS Parameters
There are two ROS parameters associated with this node. 

"INTERFACE_NAME" --> sets the device id of the network interface. By default it is set to "wlan0"

"update_rate" -> sets the message publishing frequency

## Running all nodes together

Use the provided ROS launch files (in XML format). 

The "Client.launch" should be run at the client side where the performance is measured. For instance, this would be run on a mobile robot. 

The "Server.launch" should be run at a remote station to which the client is communitating its data to. For instance, this would be a command station computer from where a robot is controlled.



on the client side - if you want to record all the metrics at the client side

```
  roslaunch network_analysis Client.launch
```

on the server side - if you just want to use for recoding delay and all other metrics at the client side only
```
  roslaunch network_analysis Server-ping-only.launch
```


on the server side - if you want to record all the metrics at the server side as well
```
  roslaunch network_analysis Server.launch
```


## Core contributors

* **Pranav Pandey** - PhD student

* **Dr.Ramviyas Parasuraman** - Principal Investigator


## Heterogeneous Robotics (HeRoLab)

Heterogeneous Robotics Research Lab (HeRoLab) of the University of Georgia.

Please contact hero at uga . edu for any queries

http://hero.uga.edu/

<p align="center">
<img src="http://hero.uga.edu/wp-content/uploads/2021/04/herolab_newlogo_whitebg.png" width="300">
</p>


