#ifndef _AFFORDANCE_TEMPLATE_H_
#define _AFFORDANCE_TEMPLATE_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <affordance_template_markers/robot_interface.h>

#include <affordance_template_library/affordance_template_structure.h>
#include <affordance_template_library/affordance_template_parser.h>

// #include <affordance_template_msgs/RobotConfig.h>
// #include <affordance_template_msgs/EndEffectorConfig.h>
// #include <affordance_template_msgs/EndEffectorPoseData.h>

namespace affordance_template 
{
  class AffordanceTemplate
  {

  public:
    
    AffordanceTemplate(const ros::NodeHandle nh, 
                       boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server, 
                       std::string robot_name, 
                       std::string template_type,
                       int id);
    ~AffordanceTemplate();

    void spin();

    void setRobotInterface(boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface);
    void setupMenuOptions();

    bool loadFromFile(std::string filename, geometry_msgs::Pose pose, affordance_template_object::AffordanceTemplateStructure &structure);


  private:

    // menu config will hold the menu text as well as bool for whether it should have a checkbox
    typedef std::pair<std::string, bool> MenuConfig;

    // this will handle menus. first item is group name, vector list is nested menu text(s)
    typedef std::map<std::string, std::vector<std::string> > MenuHandleKey;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface_;

    std::string robot_name_;
    std::string template_type_;
    int id_;
    double loop_rate_;

    std::vector<MenuConfig> object_menu_options_;
    std::vector<MenuConfig> waypoint_menu_options_;
      
    affordance_template_object::AffordanceTemplateParser at_parser_;


    std::string appendID(std::string s);
    bool appendIDToStructure(affordance_template_object::AffordanceTemplateStructure &structure);
    

  };
}

#endif