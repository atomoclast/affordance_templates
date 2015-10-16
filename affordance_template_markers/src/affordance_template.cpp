#include <affordance_template_markers/affordance_template.h>

using namespace affordance_template;
using namespace affordance_template_object;
using namespace affordance_template_markers;

AffordanceTemplate::AffordanceTemplate(const ros::NodeHandle nh, 
                                        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,  
                                        std::string robot_name, 
                                        std::string template_type) :
  nh_(nh),
  server_(server),
  robot_name_(robot_name),
  template_type_(template_type),
  loop_rate_(10.0)
{

  setupMenuOptions();

  ROS_INFO("AffordanceTemplate::init() -- Done Creating new Empty AffordanceTemplate");

}


AffordanceTemplate::~AffordanceTemplate() 
{
	
}

void AffordanceTemplate::spin()
{

  ros::AsyncSpinner spinner(1.0/loop_rate_);
  spinner.start();

  ROS_INFO("%s started.", nh_.getNamespace().c_str());  
  ROS_INFO("%s spinning.", nh_.getNamespace().c_str());
  ros::Rate loop_rate(loop_rate_);
  while(ros::ok())
  {
    loop_rate.sleep();
  }
  
}

void AffordanceTemplate::setRobotInterface(boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface)
{
  robot_interface_ = robot_interface;
}


void AffordanceTemplate::setupMenuOptions() 
{

  waypoint_menu_options_.clear();
  waypoint_menu_options_.push_back(MenuConfig("Change End-Effector Pose", false));
  waypoint_menu_options_.push_back(MenuConfig("Hide Controls", true));
  waypoint_menu_options_.push_back(MenuConfig("Add Waypoint Before", false));
  waypoint_menu_options_.push_back(MenuConfig("Add Waypoint After", false));
  waypoint_menu_options_.push_back(MenuConfig("Delete Waypoint", false));
  waypoint_menu_options_.push_back(MenuConfig("Move Forward", false));
  waypoint_menu_options_.push_back(MenuConfig("Move Back", false));

  object_menu_options_.clear();
  object_menu_options_.push_back(MenuConfig("Add Waypoint Before", false));
  object_menu_options_.push_back(MenuConfig("Add Waypoint After", false));
  object_menu_options_.push_back(MenuConfig("Reset", false));
  object_menu_options_.push_back(MenuConfig("Save", false));
  object_menu_options_.push_back(MenuConfig("Hide Controls", true));
  object_menu_options_.push_back(MenuConfig("Choose Trajectory", false));

}

bool AffordanceTemplate::loadFromFile(std::string filename, geometry_msgs::Pose pose, AffordanceTemplateStructure &structure)
{

  at_parser_.loadFromFile(filename, structure);

  // atf = open(filename).read()
  // self.structure = json.loads(atf)
  // self.structure = self.append_id_to_structure(self.structure)
  // self.load_initial_parameters(pose)
  // self.create_from_parameters()
  // stuff = filename.split("/")
  // self.filename = stuff[len(stuff)-1]
  // return self.structure
  return true;
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "affordance_template_test");
  ros::NodeHandle nh("~");
 
  std::string robot_name, template_type;
  nh.getParam("robot_name", robot_name); 
  nh.getParam("template_type", template_type); 

  boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface;
  robot_interface->load("/home/swhart/ros/catkin_workspace/src/affordance_templates/affordance_template_library/robots/r2.yaml");

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  server.reset( new interactive_markers::InteractiveMarkerServer(std::string(robot_name + "_affordance_template_server"),"",false) );

  AffordanceTemplate at(nh, server, robot_name, template_type);
  at.setRobotInterface(robot_interface);
  at.spin();
 
  return 0;
}


