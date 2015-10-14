#ifndef AT_SERVER_H_
#define AT_SERVER_H_

#include <iostream>

#include <rapidjson/document.h>
#include <rapidjson/writer.h>
#include <rapidjson/stringbuffer.h>

#include <ros/ros.h>

using namespace rapidjson;

namespace affordance_template_server
{
  class AffordanceTemplateServer
  {
  public:
    AffordanceTemplateServer();
    ~AffordanceTemplateServer();
  };
}

#endif 