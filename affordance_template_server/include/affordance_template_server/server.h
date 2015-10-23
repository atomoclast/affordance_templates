#ifndef _AFFORDANCE_TEMPLATE_SERVER_H_
#define _AFFORDANCE_TEMPLATE_SERVER_H_

#include <ros/ros.h>
#include <iostream>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <affordance_template_markers/robot_interface.h>
#include <affordance_template_markers/affordance_template.h>
#include <affordance_template_library/affordance_template_structure.h>
#include <affordance_template_library/affordance_template_parser.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <affordance_template_msgs/RobotConfig.h>
#include <affordance_template_msgs/AffordanceTemplateConfig.h>

namespace affordance_template_server
{
  class AffordanceTemplateServer
  {
    void run();
    void configureServer();
    bool loadRobot();
    bool loadTemplates();
    int getNextID(const std::string&);
    std::string getPackagePath(const std::string&);

    tf::TransformListener listener_;
    affordance_template_msgs::RobotConfig robot_config_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;
    boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface_;

    std::map<std::string, boost::shared_ptr<affordance_template::AffordanceTemplate> > at_map_;
    std::map<std::string, affordance_template_object::AffordanceTemplateStructure> at_structure_map_;

    bool status_;
    std::string pkg_name_;
    std::string robot_yaml_;
    std::string robot_name_;
    
  public:
    AffordanceTemplateServer(){} // default constructor 
    AffordanceTemplateServer(const std::string&);
    ~AffordanceTemplateServer(){}
     
    inline bool getStatus() { return status_; }
    inline void setStatus(bool status) { status_ = status; }
    inline bool findTemplate(const std::string &name) { at_structure_map_.find(name) == at_structure_map_.end() ? false : true; }
    inline affordance_template_msgs::RobotConfig getRobotConfig() { return robot_config_; }
    
    std::vector<affordance_template_msgs::AffordanceTemplateConfig> getTemplate(const std::string &name="");

    bool loadRobot(const std::string&); // from file
    bool loadRobot(const affordance_template_msgs::RobotConfig&); // from msg

    bool addTemplate(const std::string&, uint8_t&);
    bool addTemplate(const std::string&, uint8_t&, geometry_msgs::PoseStamped&);
    bool removeTemplate(const std::string&, const uint8_t);
    bool updateTemplate(const std::string&, const uint8_t, const geometry_msgs::PoseStamped&);

    bool getTemplateInstance(const std::string&, const uint8_t, boost::shared_ptr<affordance_template::AffordanceTemplate>&);
    bool getTemplateInstance(const std::string&, boost::shared_ptr<affordance_template::AffordanceTemplate>&);
  };
}

#endif 