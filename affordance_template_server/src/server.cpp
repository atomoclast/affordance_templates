#include <affordance_template_server/server.h>

using namespace affordance_template_server;

AffordanceTemplateServer::AffordanceTemplateServer()
{
    pkg_name_ = "affordance_template_library";
    std::map<std::string, affordance_template_markers::RobotInterface*> r = getAvailableRobots();
    //todo need return type
    getAvailableTemplates();

    status_ = false;
}

AffordanceTemplateServer::~AffordanceTemplateServer() {}

void AffordanceTemplateServer::configureServer()
{
    //srv_interface_ = affordance_template_server::ServiceInterface(this, listener_); // TODO service interface calss
    status_ = true;
}

void AffordanceTemplateServer::run()
{
    configureServer();
    while (ros::ok())
        ros::Duration(1).sleep();
}

void AffordanceTemplateServer::getAvailableTemplates()
{
    std::string root = getPackagePath(pkg_name_);
    if (!root.empty())
    {
        root += "/templates";
        if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
        {
            ROS_WARN("[AffordanceTemplateServer::getAvailableTemplates] cannot find templates in path %s!!", root.c_str());
            return; //TODO
        }

        // use boost to get all of the files with .yaml extension
        std::map<std::string, std::string> template_paths;
        boost::filesystem::recursive_directory_iterator dir_it(root);
        boost::filesystem::recursive_directory_iterator end_it;
        while (dir_it != end_it)
        {
            if (dir_it->path().extension() == ".json")
            {
                std::string file = dir_it->path().string();
                char *cstr = new char[file.length() + 1];
                strcpy(cstr, file.c_str());
                char* str = std::strtok(cstr, "/");

                while(std::string(str).find(".json") == std::string::npos && ros::ok())
                    str = std::strtok(NULL, "/");

                std::string name(str);
                ROS_INFO("[AffordanceTemplateServer::getAvailableTemplates] found template name: %s", name.c_str());
                template_paths[name] = dir_it->path().string();
                
                delete [] cstr;
            }
            ++dir_it;
        }

        // make robot instances with the .yamls we just found
        for (auto& t : template_paths)
        {
            ROS_INFO("[AffordanceTemplateServer::getAvailableTemplates] parsing template: %s", t.first.c_str());
            
            rapidjson::Document d;
            if (!d.Parse(t.second.c_str()).HasParseError())
            {
                
            }
            else
                ROS_WARN("[AffordanceTemplateServer::getAvailableTemplates] couldn't properly parse template; ignoring.");
        }
    }
    else
        ROS_WARN("[AffordanceTemplateServer::getAvailableTemplates] cannot find templates path!!");   
}

std::string AffordanceTemplateServer::getPackagePath(const std::string &pkg_name)
{
    ROS_INFO("[AffordanceTemplateServer::getPackagePath] finding path for package %s", pkg_name.c_str());
    std::string path = ros::package::getPath(pkg_name); // will be empty if not found
    if (path.empty())
        ROS_WARN("[AffordanceTemplateServer::getPackagePath] couldn't find path to package %s!!", pkg_name.c_str());
    else
        ROS_INFO("[AffordanceTemplateServer::getPackagePath] found path: %s", path.c_str());
    return path;
}

std::map<std::string, affordance_template_markers::RobotInterface*> AffordanceTemplateServer::getAvailableRobots()
{
    ROS_INFO("[AffordanceTemplateServer::getAvailableRobots] loading all robots from path: %s", pkg_name_.c_str());

    std::map<std::string, affordance_template_markers::RobotInterface*> robots;

    std::string root = getPackagePath(pkg_name_);
    if (!root.empty())
    {
        if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
        {
            ROS_WARN("[AffordanceTemplateServer::getAvailableRobots] cannot find robots in path: %s!!", root.c_str());
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
                ROS_INFO("[AffordanceTemplateServer::getAvailableRobots] found robot yaml at path: %s", dir_it->path().string().c_str());
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
  AffordanceTemplateServer atp;

  return 0;
}