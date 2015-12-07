#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <affordance_template_msgs/ExecuteAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_actionlib");

  ros::NodeHandle nh;
  
  // TESTTTTTT
  actionlib::SimpleActionClient<affordance_template_msgs::ExecuteAction> ac("/affordance_template/Wheel_0/execution_server", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");
  
  affordance_template_msgs::ExecuteGoal goal;
  goal.groups.push_back("left_hand");
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_ERROR("Action did not finish before the time out.");

  ros::spinOnce();

  return 0;
}