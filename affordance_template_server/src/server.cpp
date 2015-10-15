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
        // clear what we have, start over
        at_collection_.clear();

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
            FILE* f_pnt = std::fopen(t.second.c_str(), "r");
            char json_buff[65536];
            rapidjson::FileReadStream json(f_pnt, json_buff, sizeof(json_buff));

            rapidjson::Document d;
            if (d.ParseStream(json).HasParseError())
            {
                ROS_WARN("[AffordanceTemplateServer::getAvailableTemplates] couldn't properly parse template; ignoring.");
            }
            else
            {
                ROS_INFO("[AffordanceTemplateServer::getAvailableTemplates] parsing template: %s", t.first.c_str());

                affordance_template_object::AffordanceTemplateStructure at;
                at.name = d["name"].GetString();
                ROS_WARN_STREAM("name is "<<at.name);
                at.image = d["image"].GetString();
                ROS_WARN_STREAM("img is "<<at.image);
                at.filename = t.second;
                ROS_WARN_STREAM("filename is "<<at.filename);

                const rapidjson::Value& traj = d["end_effector_trajectory"];
                for (rapidjson::SizeType t = 0; t < traj.Size(); ++t)
                {
                    affordance_template_object::EndEffector ee;
                    ee.name = traj[t]["name"].GetString();
                    const rapidjson::Value& ee_group = traj[t]["end_effector_group"];
                    ee.id = ee_group[0]["id"].GetInt();
                    ROS_WARN_STREAM("    found EE trajectory with name: "<<ee.name<<" and id: "<<ee.id);

                    const rapidjson::Value& waypoints = ee_group[0]["end_effector_waypoint"];
                    for (rapidjson::SizeType w = 0; w < waypoints.Size(); ++w)
                    {
                        // find the origin
                        affordance_template_object::Origin org;
                        const rapidjson::Value& pos = waypoints[w]["origin"]["xyz"];
                        org.position[0] = pos[0].GetDouble();
                        org.position[1] = pos[1].GetDouble();
                        org.position[2] = pos[2].GetDouble();
                        const rapidjson::Value& euler = waypoints[w]["origin"]["rpy"];
                        org.orientation[0] = euler[0].GetDouble();
                        org.orientation[1] = euler[1].GetDouble();
                        org.orientation[2] = euler[2].GetDouble();

                        // create the controls
                        // ** note: it's a little cryptic but basically some of the jsons have "1" for true under controls
                        // **       so this checks to see if it's bool or int, then converts int if not bool
                        affordance_template_object::Control ctrl;
                        const rapidjson::Value& trans = waypoints[w]["controls"]["xyz"];
                        ctrl.translation[0] = trans[0].IsBool() ? trans[0].GetBool() : (trans[0].GetInt() == 1 ? true : false);
                        ctrl.translation[1] = trans[1].IsBool() ? trans[1].GetBool() : (trans[1].GetInt() == 1 ? true : false);
                        ctrl.translation[2] = trans[2].IsBool() ? trans[2].GetBool() : (trans[2].GetInt() == 1 ? true : false);
                        const rapidjson::Value& rot = waypoints[w]["controls"]["rpy"];
                        ctrl.rotation[0] = rot[0].IsBool() ? rot[0].GetBool() : (rot[0].GetInt() == 1 ? true : false);
                        ctrl.rotation[1] = rot[1].IsBool() ? rot[1].GetBool() : (rot[1].GetInt() == 1 ? true : false);
                        ctrl.rotation[2] = rot[2].IsBool() ? rot[2].GetBool() : (rot[2].GetInt() == 1 ? true : false);
                        ctrl.scale = waypoints[w]["controls"]["scale"].GetDouble();

                        // fill out the waypoint
                        affordance_template_object::Waypoint wp;
                        wp.ee_pose = waypoints[w]["ee_pose"].GetInt();
                        wp.display_object = waypoints[w]["display_object"].GetString();
                        wp.origin = org;
                        wp.controls = ctrl;

                        ee.waypoints.push_back(wp);
                        
                        ROS_WARN_STREAM("\twaypoint "<<w+1<<" has ee_pose: "<<wp.ee_pose<<" and display_object: "<<wp.display_object);
                        ROS_WARN("\t\tat origin XYZ: %g %g %g and RPY: %g %g %g", org.position[0], org.position[1], org.position[2], org.orientation[0], org.orientation[1], org.orientation[2]);
                        ROS_WARN("\t\tcontrol for axes set to: XYZ: %s %s %s", ctrl.toBoolString(ctrl.translation[0]).c_str(), ctrl.toBoolString(ctrl.translation[1]).c_str(), ctrl.toBoolString(ctrl.translation[2]).c_str());
                        ROS_WARN("\t\tcontrol for axes set to: RPY: %s %s %s", ctrl.toBoolString(ctrl.rotation[0]).c_str(), ctrl.toBoolString(ctrl.rotation[1]).c_str(), ctrl.toBoolString(ctrl.rotation[2]).c_str());
                    }

                    at.ee_trajectories.push_back(ee);
                }

                // TODO display objects from json

                std::cout<<std::endl;

                at_collection_[at.name] = at;
            }
            std::fclose(f_pnt);
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