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
This node pings the client everysecond and wait for 2 seconds before timeout. If the network is alive the network status will be 1 otherwise 0. Running this node will print "ROS Ping Action finished: SUCCEEDED" if the network is alive otherwise it will print "ROS Ping Action finished: FAILED". In the rosbag it will record network delay along with interface name, alive status, timestamp, header_seq and frame_id.

## Network throughput
This node records the throughput of the network in Mbps. It uses the interface name as a parameter ( by default it is wlan0) and records the total transmitted and recieved data packets and bytes from tcp and udp. It uses /proc/net/dev file to look for the interface name and fetch the required data from /proc/net/snmp file for each tcp and udp connection. It will print the coresponding message if the interface name does not exist or disconnected. You can use the below command to run this node.

```
  rosrun network_analysis link_utilization.py
```

This command will print "Total throughput on interface $interface_name is Transmit x Mbps and Receive y Mbps" everysecond. In the ros bag it will store the interface name, total_tx_mbps, total_rx_mbps, total_tx_bytes, total_rx_bytes, total_tx_packets, total_rx_packets along with the individaul tx and rx metrics for tcp and udp.

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
To run all the nodes use the below command. Below command will let you record all the above metrics in the rosbag simultaneously.

# on server side
```
  roslaunch network_analysis server.launch
```
# on client side

```
  roslaunch network_analysis client.launch
```
