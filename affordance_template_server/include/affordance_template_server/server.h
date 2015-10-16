#ifndef _AFFORDANCE_TEMPLATE_SERVER_H_
#define _AFFORDANCE_TEMPLATE_SERVER_H_

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

// #include <rapidjson/document.h>
// #include <rapidjson/filereadstream.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <affordance_template_msgs/RobotConfig.h>
#include <affordance_template_markers/robot_interface.h>
// #include <affordance_template_server/service_interface.h>
#include <affordance_template_library/affordance_template_structure.h>
#include <affordance_template_library/affordance_template_parser.h>

#include <interactive_markers/interactive_marker_server.h>

// using namespace rapidjson;

namespace affordance_template_server
{
  class AffordanceTemplateServer
  {
    void configureServer();
    bool loadRobots();
    bool loadTemplates();
    std::string getPackagePath(const std::string&);

    // affordance_template_server::ServiceInterface srv_interface; TODO
    tf::TransformListener listener_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> im_server_;

    std::map<std::string, affordance_template_markers::RobotInterface*> robot_interface_map_;
    std::map<std::string, affordance_template_object::AffordanceTemplateStructure> at_collection_;

    bool status_;

    std::string pkg_name_;
    
  public:
    AffordanceTemplateServer(const std::string&);
    ~AffordanceTemplateServer();
    
    void run();
    inline bool getStatus() { return status_; }
    // bool addTemplate(const std::string&);
    // bool removeTemplate(const std::string&);
    // int getNextTemplateId();
    // bool loadFromFile(const std::string&);
    // bool updateTemplatePose();
  };
}

#endif 