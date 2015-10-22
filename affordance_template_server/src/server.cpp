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
            robot_config_map_[rconf.name] = rconf;
            robot_interface_map_[ri->getRobotConfig().name] = ri;
        }
    }

    if (!loadTemplates())
        ROS_ERROR("[AffordanceTemplateServer] couldn't parse robot JSONs!!");

    status_ = false;

    ROS_INFO("[AffordanceTemplateServer] server configured. spinning...");
    at_server_thread.join();
}

AffordanceTemplateServer::~AffordanceTemplateServer() 
{
    for (auto ri : robot_interface_map_)
        delete ri.second;
    robot_interface_map_.clear();
    for (auto at : at_map_)
        delete at.second;
    at_map_.clear();
}

void AffordanceTemplateServer::run()
{
    status_ = true;
    ros::spin();
}

/**
 * @brief parse robot JSONs
 * @details finds any and all JSON files in the AT Library 'templates' directory 
 *  and parses into the map at_structure_map_ 
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
    at_structure_map_.clear();

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
        at_structure_map_[at.name] = at;
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
        for (auto ri : robot_interface_map_)
            delete ri.second;
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
                robot_config_map_[rconf.name] = rconf;
                robot_interface_map_[ri->getRobotConfig().name] = ri;
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

int AffordanceTemplateServer::getNextID(const std::string &type)
{
    std::vector<int> ids;
    for (auto at : at_map_)
    {
        if (at.second->getType() == type)
            ids.push_back(at.second->getID());
    }

    int next = 0;
    while(1)
    {
        if (std::find(ids.begin(), ids.end(), next) == ids.end())
            return next;
        else 
            ++next;
    }

    return next;
}

//################
// public methods
//################

std::vector<affordance_template_msgs::RobotConfig> AffordanceTemplateServer::getRobotConfig(const std::string &name)
{
    std::vector<affordance_template_msgs::RobotConfig> configs;
    if (!name.empty())
    {
        configs.push_back(robot_config_map_[name]);
    }
    else
    {
        for (auto c : robot_config_map_)
            configs.push_back(c.second);
    }
    return configs;
}

std::vector<affordance_template_msgs::AffordanceTemplateConfig> AffordanceTemplateServer::getTemplate(const std::string &name)
{
    std::vector<affordance_template_msgs::AffordanceTemplateConfig> templates;
    if (!name.empty())
    {
        affordance_template_msgs::AffordanceTemplateConfig atc;
        atc.filename = at_structure_map_[name].filename;
        atc.type = at_structure_map_[name].name;
        atc.image_path = at_structure_map_[name].image;
        for (auto ee : at_structure_map_[name].ee_trajectories)
        {
            affordance_template_msgs::WaypointTrajectory wp;
            wp.name = ee.name;
            for (int w = 0; w < ee.ee_waypoint_list.size(); ++w)
            {
                affordance_template_msgs::WaypointInfo wi;
                wi.id = ee.ee_waypoint_list[w].id;
                wi.num_waypoints = ee.ee_waypoint_list[w].waypoints.size();
                wp.waypoint_info.push_back(wi);
            }
            atc.trajectory_info.push_back(wp);
        }
        for (auto d : at_structure_map_[name].display_objects)
            atc.display_objects.push_back(d.name);        
        templates.push_back(atc);
    }
    else
    {   
        for (auto t : at_structure_map_)
        {
            affordance_template_msgs::AffordanceTemplateConfig atc;
            atc.filename = t.second.filename;
            atc.type = t.second.name;
            atc.image_path = t.second.image;
            for (auto ee : t.second.ee_trajectories)
            {
                affordance_template_msgs::WaypointTrajectory wp;
                wp.name = ee.name;
                for (int w = 0; w < ee.ee_waypoint_list.size(); ++w)
                {
                    affordance_template_msgs::WaypointInfo wi;
                    wi.id = ee.ee_waypoint_list[w].id;
                    wi.num_waypoints = ee.ee_waypoint_list[w].waypoints.size();
                    wp.waypoint_info.push_back(wi);
                }
                atc.trajectory_info.push_back(wp);
            }
            for (auto d : t.second.display_objects)
                atc.display_objects.push_back(d.name);
            templates.push_back(atc);
        }
    }
    return templates;
}

bool AffordanceTemplateServer::loadRobot(const std::string &name="")
{
    if (!name.empty())
        return false;

    robot_interface_map_[name]->tearDown();
    return robot_interface_map_[name]->load(name);
} 

bool AffordanceTemplateServer::loadRobot(const affordance_template_msgs::RobotConfig &msg)
{
    std::string name = msg.name;
    robot_interface_map_[name]->tearDown();
    return robot_interface_map_[name]->load(msg);
}

bool AffordanceTemplateServer::addTemplate(const std::string &type, uint8_t& id, geometry_msgs::PoseStamped &pose)
{
    if (type.empty())
        return false;

    id = getNextID(type);
    std::string key = type + ":" + std::to_string(id);
    ROS_INFO("[AffordanceTemplateServer::addTemplate] creating new affordance template with ID: %d and key: %s", id, key.c_str());

    // at_map_[key] = new affordance_template::AffordanceTemplate(ros::NodeHandle, im_server_, robot_interface_map_[""], "some_goddammed_robot", type, id); // WTF is the robot name??

    return true;
}

bool AffordanceTemplateServer::removeTemplate(const std::string &type, const uint8_t id)
{
    std::string key = type + ":" + std::to_string(id);
    if (at_map_.find(key) == at_map_.end())
        return false;

    delete at_map_[key];
    at_map_.erase(key);

    return true;
}

bool AffordanceTemplateServer::getTemplateInstance(const std::string &type, const uint8_t id, affordance_template::AffordanceTemplate* ati)
{
   std::string key = type + ":" + std::to_string(id);
    if (at_map_.find(key) == at_map_.end())
        return false;

    ati = at_map_[key];

    return true;
}