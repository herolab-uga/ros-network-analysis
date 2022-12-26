// Description: This ROS node is the server node which provides action requested by the network_delay node. 
// This helps in calculating the application-level delay in the (wireless) network.
// Author: Ramviyas Parasuraman ramviyas@uga.edu
// License: MIT
// Target executable file name - pingactionserver

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <network_analysis/PingAction.h>

class PingAction : public rclcpp::Node
{
protected:

  // auto node = rclcpp::Node::make_shared("pingactionserver");
  rclcpp_action::Server<network_analysis::PingAction>::SharedPtr as_; 
  std::string action_name_;
  network_analysis::PingFeedback feedback_;
  network_analysis::PingResult result_;

public:

  PingAction(std::string name):
    action_name_(name)
  {
    this->as_ = rclcpp_action::create_server<network_analysis::PingAction>(
      this,
      action_name_,
      std::bind(&PingAction::executeCB, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

  ~PingAction(void)
  {
  }

  void executeCB(const network_analysis::PingGoalConstPtr &goal)
  {
    // helper variables
    rclcpp::Rate r(1000); // have to do this action every ms
    bool success = true;

	// start executing the action
    if(goal->inp)
    {
      feedback_.fb = true;
      // publish the feedback
      this->as_publishFeedback(feedback_);
      //r.sleep  
    }

    if(success)
    {
      result_.outp = feedback_.fb;
      RCLCPP_INFO(this->get_logger(), "ROS Action %s: Succeeded\n", action_name_.c_str());
      // set the action state to succeeded
      this->as_succeed(result_);
    }
  }
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  PingAction Ping("Ping");
  rclcpp::spin(ping);

  return 0;
}
