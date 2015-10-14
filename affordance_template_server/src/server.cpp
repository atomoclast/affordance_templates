#include <affordance_template_server/server.h>

using namespace affordance_template_server;

AffordanceTemplateServer::AffordanceTemplateServer()
{
  const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
  rapidjson::Document d;
  d.Parse(json);
}

AffordanceTemplateServer::~AffordanceTemplateServer() {}


// tester main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "at_json_parser_test_main");
  AffordanceTemplateServer atp();

  return 0;
}