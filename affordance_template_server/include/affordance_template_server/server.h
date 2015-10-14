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

using namespace rapidjson;

namespace affordance_template_server
{
  class AffordanceTemplateServer
  {
    void run();
    std::map<std::string, affordance_template_markers::RobotInterface&> getAvailableRobots(const std::string&);
    inline bool getStatus() { return status_; }

    tf::TransformListener listener_;

    bool status_;
  
  public:
    AffordanceTemplateServer();
    ~AffordanceTemplateServer();
    
    // void configureServer();
    // bool addTemplate();
    // bool removeTemplate();
    // int getNextTemplateId();
    std::string getPackagePath(const std::string&);
    // ?? getAvailableTemplates(); TODO
    // bool loadFromFile(const std::string&);
    // bool updateTemplatePose();
  };
}

#endif 