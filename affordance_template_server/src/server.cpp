#include <affordance_template_server/server.h>

using namespace affordance_template_server;

AffordanceTemplateServer::AffordanceTemplateServer(const std::string &_robot_yaml="")
{
    boost::thread at_server_thread(boost::bind(&AffordanceTemplateServer::run, this));

    if (_robot_yaml.empty())
        ROS_WARN("[AffordanceTemplateServer] no robot yaml provided - BE SURE TO LOAD ROBOT FROM SERVICE!!");

    pkg_name_ = "affordance_template_library";

    im_server_.reset( new interactive_markers::InteractiveMarkerServer(std::string("affordance_template_interactive_marker_server"), "", false));
    
    // load up robots if yaml provided
    if (!_robot_yaml.empty())
        if (!loadRobots())
            ROS_ERROR("[AffordanceTemplateServer] couldn't parse robot .yamls!!");
    else 
    {
        affordance_template_markers::RobotInterface *ri = new affordance_template_markers::RobotInterface();
        bool loaded = ri->load(_robot_yaml);

        if (!loaded)        
            ROS_WARN("[AffordanceTemplateServer] robot %s NOT loaded", _robot_yaml.c_str());

        affordance_template_msgs::RobotConfig rconf = ri->getRobotConfig();
        if (getPackagePath(rconf.config_package).empty())
        {
            ROS_WARN("[AffordanceTemplateServer::loadRobots] config package %s NOT found, ignoring.", _robot_yaml.c_str());
        }
        else
        {
            ROS_INFO("[AffordanceTemplateServer::loadRobots] config package %s found", _robot_yaml.c_str());
            robot_interface_map_[_robot_yaml] = ri;
        }
    }

    if (!loadTemplates())
        ROS_ERROR("[AffordanceTemplateServer] couldn't parse robot JSONs!!");

    status_ = false;

    ROS_INFO("[AffordanceTemplateServer] server configured. spinning...");
    at_server_thread.join();
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
    ros::spin();
}

/**
 * @brief parse robot JSONs
 * @details finds any and all JSON files in the AT Library 'templates' directory 
 *  and parses into the map at_collection_ 
 * 
 * @return false if can't find or load the directory, true otherwise
 */
bool AffordanceTemplateServer::loadTemplates()
{
    ROS_INFO("[AffordanceTemplateServer:loadTemplates] searching for JSON templates in package: %s", pkg_name_.c_str());

    affordance_template_object::AffordanceTemplateParser atp;

    std::string root = getPackagePath(pkg_name_);
    if (root.empty())
        return false;
    
    // clear what we have, start over
    at_collection_.clear();

    root += "/templates";
    if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
    {
        ROS_WARN("[AffordanceTemplateServer::loadTemplates] cannot find templates in path %s!!", root.c_str());
        return false;
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
            ROS_INFO("[AffordanceTemplateServer::loadTemplates] found template name: %s", name.c_str());
            template_paths[name] = dir_it->path().string();
            
            delete [] cstr;
        }
        ++dir_it;
    }

    // make robot instances with the .yamls we just found
    for (auto& t : template_paths)
    {
        affordance_template_object::AffordanceTemplateStructure at;
        atp.loadFromFile(t.second, at);
        at_collection_[at.name] = at;
    }

    return true;
}

bool AffordanceTemplateServer::loadRobots()
{
    ROS_INFO("[AffordanceTemplateServer::loadRobots] loading all robots from path: %s", pkg_name_.c_str());

    std::string root = getPackagePath(pkg_name_);
    if (!root.empty())
    {
        if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
        {
            ROS_WARN("[AffordanceTemplateServer::loadRobots] cannot find robots in path: %s!!", root.c_str());
            return false;
        }

        // use boost to get all of the files with .yaml extension
        std::vector<std::string> robot_paths_vec;
        boost::filesystem::recursive_directory_iterator dir_it(root);
        boost::filesystem::recursive_directory_iterator end_it;
        while (dir_it != end_it)
        {
            if (boost::filesystem::is_regular_file(*dir_it) && dir_it->path().extension() == ".yaml")
            {
                ROS_INFO("[AffordanceTemplateServer::loadRobots] found robot yaml at path: %s", dir_it->path().string().c_str());
                robot_paths_vec.push_back(dir_it->path().string());
            }
            ++dir_it;
        }

        // make robot instances with the .yamls we just found
        robot_interface_map_.clear();
        for (auto r : robot_paths_vec)
        {
            affordance_template_markers::RobotInterface *ri = new affordance_template_markers::RobotInterface();
            bool loaded = ri->load(r);

            if (!loaded)
            {
                ROS_WARN("[AffordanceTemplateServer::loadRobots] robot %s NOT loaded, ignoring.", r.c_str());
                continue;
            }

            affordance_template_msgs::RobotConfig rconf = ri->getRobotConfig();
            if (getPackagePath(rconf.config_package).empty())
            {
                ROS_WARN("[AffordanceTemplateServer::loadRobots] config package %s NOT found, ignoring.", r.c_str());
            }
            else
            {
                ROS_INFO("[AffordanceTemplateServer::loadRobots] config package %s found", r.c_str());
                robot_interface_map_[r] = ri;
            }
        }
    }
    else
        ROS_WARN("[AffordanceTemplateServer::loadRobots] cannot find robots path!!");

    return true;
}

std::string AffordanceTemplateServer::getPackagePath(const std::string &pkg_name)
{
    ROS_INFO("[AffordanceTemplateServer::getPackagePath] finding path for package %s.......", pkg_name.c_str());
    std::string path = ros::package::getPath(pkg_name); // will be empty if not found
    if (path.empty())
        ROS_WARN("[AffordanceTemplateServer::getPackagePath] couldn't find path to package %s!!", pkg_name.c_str());
    else
        ROS_INFO("[AffordanceTemplateServer::getPackagePath] found path: %s", path.c_str());
    return path;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "affordance_template_server");

  ros::NodeHandle nh;
  std::string robot_name = "";
  nh.getParam("robot_config", robot_name);

  affordance_template_server::AffordanceTemplateServer ats(robot_name);

  return 0;
}