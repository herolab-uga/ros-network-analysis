// Description: This ROS node calculates delay in the network without using Ping/ICMP echo requests. It uses ROS action calls instead, thus measuring the round trip time (2xlatency) at the applicaiton level layer (with TCP connection).
// Author: Ramviyas Parasuraman ramviyas@uga.edu
// License: MIT


#include <ros/ros.h>
#include <network_analysis/PingAction.h>
#include <network_analysis/NetworkDelay.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sys/time.h>
#include <iostream>

int main(int argc, char* argv[])
{
	try
	{
		ros::init(argc, argv, "network_delay");

		ros::NodeHandle n;

		struct timeval start, end;
		double mtime, seconds, useconds;
		double timeout=2.0;
		int updaterate=1;
		n.param("~timeout_network_delay", timeout, 2.0); // timeout to wait for action (in seconds)
		n.param("~update_rate_network_delay", updaterate, 1); // update freqency to publish in the topic

		// Publish the delay values in the /network_analysis/network_delay topic
		ros::Publisher pub = n.advertise<network_analysis::NetworkDelay>("network_analysis/network_delay", 10);
		network_analysis::NetworkDelay msg;

		// delay calculation using ROS action calls
		actionlib::SimpleActionClient<network_analysis::PingAction> ac("Ping", true);
		ac.waitForServer(); //will wait for infinite time until the pingactionserver is started
		network_analysis::PingGoal goal;
		goal.inp = true;

		ros::Rate loop_rate(updaterate);
		//ros::Time last_time = ros::Time::now();

		msg.header.stamp = ros::Time::now();
		msg.iface = "default";
		msg.delay = -1; //default values
		msg.alive = 0; //default values

		while (ros::ok())
		{
			gettimeofday(&start, NULL);
			ac.sendGoal(goal);
			//wait for the action to return
			bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout));
			if (finished_before_timeout)
			  {
				actionlib::SimpleClientGoalState state = ac.getState();
				ROS_INFO("ROS Ping Action finished: %s",state.toString().c_str());
				gettimeofday(&end, NULL);
				seconds  = end.tv_sec  - start.tv_sec;
				useconds = end.tv_usec - start.tv_usec;
				mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
				msg.delay = (float) mtime;
				msg.alive = 1;
			  }
			else
			  {
				ROS_ERROR("Action did not finish before the timeout. May be a network problem or may be the ping action server node stopped. See network_delay.alive history for more diagnosis");
				msg.delay=-1; //error values
				msg.alive=0; //error values
			  }
			pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	catch (std::exception& e)
	{
	  std::cerr << "Exception: " << e.what() << std::endl;
	}

	return 0;
}
