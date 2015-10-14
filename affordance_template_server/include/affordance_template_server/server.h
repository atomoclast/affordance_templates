#ifndef _AT_SERVER_H_
#define _AT_SERVER_H_

#include <iostream>
#include <boost/filesystem.hpp>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <affordance_template_msgs/RobotConfig.h>
#include <affordance_template_markers/robot_interface.h>
// #include <affordance_template_server/service_interface.h>
#include <affordance_template_library/affordance_template_collection.h>

using namespace rapidjson;

namespace affordance_template_server
{
  class AffordanceTemplateServer
  {
    void configureServer();
    std::map<std::string, affordance_template_markers::RobotInterface*> getAvailableRobots();
    void getAvailableTemplates(); //std::map<std::string, affordance_template_object::AffordanceTemplateCollection>
    inline bool getStatus() { return status_; }

    // affordance_template_server::ServiceInterface srv_interface; TODO
    tf::TransformListener listener_;

    bool status_;

    std::string pkg_name_;
    
  public:
    AffordanceTemplateServer(); //TODO
    ~AffordanceTemplateServer();
    
    void run();
    // bool addTemplate();
    // bool removeTemplate();
    // int getNextTemplateId();
    std::string getPackagePath(const std::string&);
    // bool loadFromFile(const std::string&);
    // bool updateTemplatePose();
  };
}

#endif 