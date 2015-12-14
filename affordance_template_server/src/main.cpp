#include <affordance_template_server/interface.h>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "affordance_template_server_node");
  ros::AsyncSpinner spinner(10);
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;
    std::string robot_name = "";

  pnh.getParam("robot_config", robot_name);

  ROS_WARN("ROBOT NAME: %s", robot_name.c_str());
  if (robot_name.empty())
    robot_name = "r2_roscontrol.yaml";

  affordance_template_server::AffordanceTemplateInterface ati(nh, robot_name);

  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}