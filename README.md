# ROS Network Analysis Package
This is a ROS package that provide tools to analyze the wireless network such as the signal quality, latency, throughput, link utilization, connection rates, error metrics, etc., between two ROS nodes/computers/machines.

#Installation
To use these packages you will have to install the package into your ros package. Below are the commands which will help you do that, considering you already have catkin workspace.
  
  cd ~/catkin_ws
  cd src
  git clone 
  cd ../..
  catkin_make
  catkin_make install
  
#Use Packages

Apart from network delay package, other two packages need two ROS nodes, One as listener and another as speaker. There are four packages linkUtiliaztion, NetworkErrors, wireless_quality and network_delay. Below are the commands to use them individually.

linkUtilization.py
This node study the throughput of the interfaces being used. Change the interface name in linkutilization.py file according to the interface you are using (by default it will be wlan0). This needs two ROS nodes : one for transmitting and one for receiving data. Based on the transfer rate you will get the throughput. This will provide the throughput in mbps along with the bytes and data packets transfered between the two nodes.

  roslaunch network_analysis

networkerrors.py
This node track the error package being retransmitted along with the other metrics value like retires.

  roslaunch network_analysis

wireless_quality.py
This node records the RSSI value along with lqi value for the interface being used.

network_delay.py
This node determines the delay of the interface passed. From this node we can also find out the percentage of temporary disconnection that happened while using the interface. This does not need two nodes as it only gives the delay for the network to which your ROS node is connected. 
