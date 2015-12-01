#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <affordance_template_msgs/PlanAction.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_actionlib");

  ros::NodeHandle nh;
  
  // TESTTTTTT
  actionlib::SimpleActionClient<affordance_template_msgs::PlanAction> ac("/affordance_template/TestWheel_0/planning_server", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");
  
  affordance_template_msgs::PlanGoal goal;
  goal.ee.push_back("left_hand");
  goal.steps = 1;
  goal.planning = affordance_template_msgs::PlanGoal::CARTESIAN;
  goal.backwards = false;
  goal.execute_on_plan = false;
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