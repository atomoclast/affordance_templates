#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <affordance_template_msgs/PlanAction.h>

// - new way:
//       geometry_msgs::PoseStamped pt;
//       pt = frame_store_[next_path_str + "/tf"].second;
//       goals[manipulator_name].push_back(pt);
// - or goals[manipulator_name].push_back(frame_store_[next_path_str + "/tf"].second);

// because it has to execute to the next waypoint, then execute the grasp pose, then loop if more then one step
// oh the Goal needs forward or backwards too, and if its moving straight to the first or last wp of the traj
// guess it needs the trajectory name too
// so that's the basic idea

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_actionlib");

  ros::NodeHandle nh;
  
  // TESTTTTTT
  actionlib::SimpleActionClient<affordance_template_msgs::PlanAction> ac("/affordance_template/Wheel_0/planning_server", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); //will wait for infinite time
  ROS_INFO("Action server started, sending goal.");
  
  affordance_template_msgs::PlanGoal goal;
  goal.groups.push_back("left_hand"); // should be trajectory?? and have AT figure out which groups are in thet rajectory??
  goal.steps = 1; // 0 to plan for all steps
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