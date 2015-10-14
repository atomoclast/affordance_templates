#include <affordance_template_server/server.h>

using namespace affordance_template_server;

AffordanceTemplateServer::AffordanceTemplateServer()
{
  const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
  rapidjson::Document d;
  d.Parse(json);
}

AffordanceTemplateServer::~AffordanceTemplateServer() {}


std::string AffordanceTemplateServer::getPackagePath(const std::string &pkg_name)
{
    ROS_INFO("[AffordanceTemplateServer::getPackagePath]  finding path for package %s", pkg_name.c_str());
    std::string path = ros::package::getPath(pkg_name); // will be empty if not found
    if (path.empty())
        ROS_WARN("[AffordanceTemplateServer::getPackagePath] couldn't find path to package %s!!", pkg_name.c_str());
    else
        ROS_INFO("[AffordanceTemplateServer::getPackagePath] found path %s", path.c_str());
    return path;
}

std::map<std::string, affordance_template_markers::RobotInterface*> AffordanceTemplateServer::getAvailableRobots(const std::string &path)
{
    ROS_INFO("[AffordanceTemplateServer::getAvailableRobots] loading all robots from path %s", path.c_str());

    std::map<std::string, affordance_template_markers::RobotInterface*> robots;

    std::string root = ros::package::getPath(path);
    if (!root.empty())
    {
        if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
        {
            ROS_WARN("[AffordanceTemplateServer::getAvailableRobots] cannot find robots in path %s!!", root.c_str());
            return robots;
        }

        // use boost to get all of the files with .yaml extension
        std::vector<std::string> robot_paths_vec;
        boost::filesystem::recursive_directory_iterator dir_it(root);
        boost::filesystem::recursive_directory_iterator end_it;
        while (dir_it != end_it)
        {
            if (boost::filesystem::is_regular_file(*dir_it) && dir_it->path().extension() == ".yaml")
            {
                ROS_INFO("[AffordanceTemplateServer::getAvailableRobots] found robot yaml path %s", dir_it->path().string().c_str());
                robot_paths_vec.push_back(dir_it->path().string());
            }
            ++dir_it;
        }

        // make robot instances with the .yamls we just found
        for (auto r : robot_paths_vec)
        {
            affordance_template_markers::RobotInterface *ri = new affordance_template_markers::RobotInterface();
            bool loaded = ri->load(r);

            if (!loaded)
            {
                ROS_WARN("[AffordanceTemplateServer::getAvailableRobots] robot %s NOT loaded, ignoring.", r.c_str());
                continue;
            }

            affordance_template_msgs::RobotConfig rconf = ri->getRobotConfig();
            if (getPackagePath(rconf.config_package).empty())
            {
                ROS_WARN("[AffordanceTemplateServer::getAvailableRobots] config package %s NOT found, ignoring.", r.c_str());
            }
            else
            {
                ROS_INFO("[AffordanceTemplateServer::getAvailableRobots] config package %s found", r.c_str());
                robots[r] = ri;
            }
        }
    }
    else
        ROS_WARN("[AffordanceTemplateServer::getAvailableRobots] cannot find robots path!!");

    return robots;
}

// tester main
int main(int argc, char **argv)
{
  ros::init(argc, argv, "at_json_parser_test_main");
  AffordanceTemplateServer atp();

  return 0;
}