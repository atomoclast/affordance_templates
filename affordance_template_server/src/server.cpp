#include <affordance_template_server/parser.h>

using namespace affordance_template_server;

ATParser::ATParser()
{
  const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
  rapidjson::Document d;
  d.Parse(json);
}

ATParser::~ATParser() {}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "at_json_parser_test_main");
  ATParser atp();

  return 0;
}