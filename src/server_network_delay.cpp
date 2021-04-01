// Description: This ROS node is the server node which provides action requested by the network_delay node. This helps in calculating the application-level delay in the (wireless) network.
// Author: Ramviyas Parasuraman ramviyas@uga.edu
// License: MIT
// Target executable file name - pingactionserver

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <network_analysis/PingAction.h>

class PingAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<network_analysis::PingAction> as_; 
  std::string action_name_;
  network_analysis::PingFeedback feedback_;
  network_analysis::PingResult result_;

public:

  PingAction(std::string name) :
    as_(nh_, name, boost::bind(&PingAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~PingAction(void)
  {
  }

  void executeCB(const network_analysis::PingGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1000); // have to do this action every ms
    bool success = true;

	// start executing the action
    if(goal->inp)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("ROS Action %s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        return;
      }
      feedback_.fb = true;
      // publish the feedback
      as_.publishFeedback(feedback_);
      //r.sleep  
    }

    if(success)
    {
      result_.outp = feedback_.fb;
      ROS_INFO("ROS Action %s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pingactionserver");

  PingAction Ping("Ping");
  ros::spin();

  return 0;
}
