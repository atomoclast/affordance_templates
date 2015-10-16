#include <affordance_template_server/interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "affordance_template_server_node");

  ros::NodeHandle nh;
  std::string robot_name = "";
  nh.getParam("robot_config", robot_name);

  affordance_template_server::AffordanceTemplateInterface ati();

  // affordance_template_server::AffordanceTemplateServer atp(robot_name);

  return 0;
}