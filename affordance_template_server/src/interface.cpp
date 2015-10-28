#include <affordance_template_server/interface.h>

using namespace affordance_template_server;
using namespace affordance_template_msgs;
using namespace affordance_template_object;

typedef boost::shared_ptr<affordance_template::AffordanceTemplate> ATPointer;

AffordanceTemplateInterface::AffordanceTemplateInterface(const std::string &_robot_yaml)
{
    ROS_INFO("[AffordanceTemplateInterface] starting...");

    ros::NodeHandle nh;

    if (!_robot_yaml.empty())
        ROS_INFO("[AffordanceTemplateInterface] creating server using robot yaml %s", _robot_yaml.c_str());
    at_server_.reset(new AffordanceTemplateServer(_robot_yaml));

    const std::string base_srv = "/affordance_template_server/";
    at_srv_map_["get_robots"]              = nh.advertiseService(base_srv + "get_robots", &AffordanceTemplateInterface::handleRobotRequest, this);
    at_srv_map_["get_templates"]           = nh.advertiseService(base_srv + "get_templates", &AffordanceTemplateInterface::handleTemplateRequest, this);
    at_srv_map_["load_robot"]              = nh.advertiseService(base_srv + "load_robot", &AffordanceTemplateInterface::handleLoadRobot, this);
    at_srv_map_["add_template"]            = nh.advertiseService(base_srv + "add_template", &AffordanceTemplateInterface::handleAddTemplate, this);
    at_srv_map_["delete_template"]         = nh.advertiseService(base_srv + "delete_template", &AffordanceTemplateInterface::handleDeleteTemplate, this);
    at_srv_map_["get_running"]             = nh.advertiseService(base_srv + "get_running", &AffordanceTemplateInterface::handleRunning, this);
    at_srv_map_["plan_command"]            = nh.advertiseService(base_srv + "plan_command", &AffordanceTemplateInterface::handlePlanCommand, this);
    at_srv_map_["execute_command"]         = nh.advertiseService(base_srv + "execute_command", &AffordanceTemplateInterface::handleExecuteCommand, this);
    at_srv_map_["save_template"]           = nh.advertiseService(base_srv + "save_template", &AffordanceTemplateInterface::handleSaveTemplate, this);
    at_srv_map_["add_trajectory"]          = nh.advertiseService(base_srv + "add_trajectory", &AffordanceTemplateInterface::handleAddTrajectory, this);
    at_srv_map_["scale_object"]            = nh.advertiseService(base_srv + "scale_object", &AffordanceTemplateInterface::handleObjectScale, this);
    at_srv_map_["get_template_status"]     = nh.advertiseService(base_srv + "get_template_status", &AffordanceTemplateInterface::handleTemplateStatus, this);
    at_srv_map_["get_status"]              = nh.advertiseService(base_srv + "get_status", &AffordanceTemplateInterface::handleServerStatus, this);
    at_srv_map_["set_template_trajectory"] = nh.advertiseService(base_srv + "set_template_trajectory", &AffordanceTemplateInterface::handleSetTrajectory, this);
    at_srv_map_["set_template_pose"]       = nh.advertiseService(base_srv + "set_template_pose", &AffordanceTemplateInterface::handleSetPose, this);
    ROS_INFO("[AffordanceTemplateInterface] services set up...");

    ROS_INFO("[AffordanceTemplateInterface] robot ready!!");
}

bool AffordanceTemplateInterface::handleRobotRequest(GetRobotConfigInfo::Request &req, GetRobotConfigInfo::Response &res)
{
    at_server_->setStatus(false);
    
    ROS_INFO("[AffordanceTemplateInterface::handleRobotRequest] requesting robot configuration from server");
    res.robots.push_back( at_server_->getRobotConfig());
    if (res.robots.size() == 0)
        ROS_WARN("[AffordanceTemplateInterface::handleRobotRequest] couldn't find any robot configurations on server!!");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleTemplateRequest(GetAffordanceTemplateConfigInfo::Request &req, GetAffordanceTemplateConfigInfo::Response &res)
{
    at_server_->setStatus(false);
    
    std::vector<AffordanceTemplateConfig> templates;
    if (!req.name.empty() && at_server_->findTemplate(req.name))
    {
        ROS_INFO("[AffordanceTemplateInterface::handleTemplateRequest] requesting %s template info", req.name.c_str());
        templates = at_server_->getAvailableTemplates(req.name);
        if (templates.size())
            res.templates.push_back(templates.front());
        else
            ROS_WARN("[AffordanceTemplateInterface::handleTemplateRequest] couldn't find template on server matching template name: %s on server!!", req.name.c_str());
    }
    else
    {
        ROS_INFO("[AffordanceTemplateInterface::handleTemplateRequest] requesting info on all loaded templates");
        templates = at_server_->getAvailableTemplates();
        if (templates.size())
            res.templates = templates;
        else
            ROS_WARN("[AffordanceTemplateInterface::handleTemplateRequest] couldn't find any templates on server!!");
    }
    
    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleLoadRobot(LoadRobotConfig::Request &req, LoadRobotConfig::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleLoadRobot] loading robot %s", req.robot_config.name.c_str());

    res.status = false;
    if (!req.filename.empty())
        res.status = at_server_->loadRobot(req.filename);
    else
        res.status = at_server_->loadRobot(req.robot_config);

    if (!res.status)
        ROS_ERROR("[AffordanceTemplateInterface::handleLoadRobot] error loading robot!!");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleAddTemplate(AddAffordanceTemplate::Request &req, AddAffordanceTemplate::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleAddTemplate] adding template: %s", req.class_type.c_str());

    res.status = at_server_->addTemplate(req.class_type, res.id, req.pose);
    ROS_INFO("[AffordanceTemplateInterface::handleAddTrajectory] added template: %s",(req.class_type+":"+std::to_string(res.id)).c_str());

    if (!res.status)
        ROS_ERROR("[AffordanceTemplateInterface::handleAddTemplate] error adding template!!");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleDeleteTemplate(DeleteAffordanceTemplate::Request &req, DeleteAffordanceTemplate::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleDeleteTemplate] removing template: %s", req.class_type.c_str());

    res.status = at_server_->removeTemplate(req.class_type, req.id);

    if (!res.status)
        ROS_ERROR("[AffordanceTemplateInterface::handleDeleteTemplate] error removing template!!");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleRunning(GetRunningAffordanceTemplates::Request &req, GetRunningAffordanceTemplates::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleRunning] gathering names of running templates");

    std::vector<std::string> templates = at_server_->getRunningTemplates();
    for (auto t : templates)
    {
        ROS_INFO("[AffordanceTemplateInterface::handleRunning] \tfound template: %s", t.c_str());
        res.templates.push_back(t);
    }

    if (templates.size() == 0)
        ROS_INFO("[AffordanceTemplateInterface::handleRunning] no templates are currently running on server");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handlePlanCommand(AffordanceTemplatePlanCommand::Request &req, AffordanceTemplatePlanCommand::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handlePlanCommand] new plan request for %s:%d, trajectory %s", req.type.c_str(), req.id, req.trajectory_name.c_str());

    ATPointer at;
    if ( !at_server_->getTemplateInstance(req.type, req.id, at))
        ROS_ERROR("[AffordanceTemplateInterface::handlePlanCommand] error getting instance of affordance template %s:%d", req.type.c_str(), req.id);
    else
    {
        // check if specific trajectory was given
        if (req.trajectory_name.empty())
            req.trajectory_name = at->getCurrentTrajectory();

        // go through all the EE waypoints in the request
        int id = 0; 
        int steps = 0;

        std::vector<std::string> ee_names;
        std::map<std::string, bool> ee_path;
        for (auto ee : req.end_effectors)
        {
            ee_path[ee] = false;
            steps = req.steps[id];
            // make sure EE is in trajectory
            if ( !at->trajectoryHasEE(req.trajectory_name, ee))
            {
                ROS_WARN("[AffordanceTemplateInterface::handlePlanCommand] %s not in trajectory, can't plan!!", ee.c_str());
                continue;
            }
            ee_names.push_back(ee);
            ++id;
        }

        // compute path plan (returns dictionary of bools keyed off EE name)
        ee_path = at->planPathToWaypoints(ee_names, steps, req.direct, req.backwards);
        ROS_INFO("[AffordanceTemplateInterface::handlePlanCommand] planned path for %lu end-effectors:", ee_path.size());
        for (auto ee : ee_path)
            ROS_INFO("[AffordanceTemplateInterface::handlePlanCommand] \t%s : %s", ee.first.c_str(), boolToString(ee.second).c_str());
        res.affordance_template_status = getTemplateStatus(req.type, req.id, req.trajectory_name);
    }

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleExecuteCommand(AffordanceTemplateExecuteCommand::Request &req, AffordanceTemplateExecuteCommand::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleExecuteCommand] new execution request for %s:%d, trajectory %s", req.type.c_str(), req.id, req.trajectory_name.c_str());

    res.status = false;

    ATPointer at;
    if ( !at_server_->getTemplateInstance(req.type, req.id, at))
        ROS_ERROR("[AffordanceTemplateInterface::handleExecuteCommand] error getting instance of affordance template %s:%d", req.type.c_str(), req.id);
    else
    {
        // check if specific trajectory was given
        if (req.trajectory_name.empty())
            req.trajectory_name = at->getCurrentTrajectory();

        std::vector<std::string> ee_list;
        for (auto ee : req.end_effectors)
        {
            if ( !at->trajectoryHasEE(req.trajectory_name, ee))
                ROS_WARN("[AffordanceTemplateInterface::handleExecuteCommand] %s not in trajectory, can't execute!!", ee.c_str());
            else
                ee_list.push_back(ee);
        }

        // if the AT has prevously computed a valid plan (can't execute unless this is True) 
        if ( at->validWaypointPlan(ee_list, req.trajectory_name))
        {
            if ( at->moveToWaypoints(ee_list))
            {
                res.status = true; // only try if all plans valid
                ROS_INFO("[AffordanceTemplateInterface::handleExecuteCommand] done executing %s plan(s)", req.trajectory_name.c_str());
            }
        }
        else
        {
            ROS_WARN("[AffordanceTemplateInterface::handleExecuteCommand] no valid plan found for %s", req.trajectory_name.c_str());
        }
        res.affordance_template_status = getTemplateStatus(req.type, req.id, req.trajectory_name);
    }

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleSaveTemplate(SaveAffordanceTemplate::Request &req, SaveAffordanceTemplate::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleSaveTemplate] saving %s:%d as %s:%d to %s with image: %s", req.original_class_type.c_str(), req.id, req.new_class_type.c_str(), req.id, req.filename.c_str(), req.image.c_str());

    res.status = false;
    std::string old_key = req.original_class_type + ":" + std::to_string(req.id);
    std::string new_key = req.new_class_type + ":" + std::to_string(req.id);
    bool save_status = false;
    ATPointer at;
    if ( at_server_->getTemplateInstance(req.original_class_type, req.id, at))
        save_status = at->saveToDisk(req.filename, req.image, new_key, req.save_scale_updates);
    bool remove_status = at_server_->removeTemplate(req.original_class_type, req.id);
    bool add_status = at_server_->addTemplate(req.new_class_type, req.id);
    res.status = (save_status && remove_status && add_status);
    if (!res.status)
        ROS_ERROR("[AffordanceTemplateInterface::handleSaveTemplate] error saving template. save to file was: %s, remove was: %s, adding was: %s", successToString(save_status).c_str(), successToString(remove_status).c_str(), successToString(add_status).c_str());

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleAddTrajectory(AddAffordanceTemplateTrajectory::Request &req, AddAffordanceTemplateTrajectory::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleAddTrajectory] adding [%s] to template %s:%d", req.trajectory_name.c_str(), req.class_type.c_str(), req.id);

    res.status = false;

    ATPointer at;
    if ( at_server_->getTemplateInstance(req.class_type, req.id, at))
        res.status = at->addTrajectory(req.trajectory_name);

    if ( !res.status )
        ROS_ERROR("[AffordanceTemplateInterface::handleAddTrajectory] error adding trajectory to template!!");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleObjectScale(ScaleDisplayObject::Request &req, ScaleDisplayObject::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleObjectScale] scaling %s:%d object[%s] by (%g,%g)", req.scale_info.class_type.c_str(), req.scale_info.id, req.scale_info.object_name.c_str(), req.scale_info.scale_factor, req.scale_info.end_effector_scale_factor);

    res.status = false;

    ATPointer at;
    // if ( at_server_->getTemplateInstance(req.scale_info.class_type, req.scale_info.id, at))
        // res.status = at->scaleObject(req.scale_info.object_name, req.scale_info.scale_factor, req.scale_info.end_effector_scale_factor);

    if ( !res.status )
        ROS_ERROR("[AffordanceTemplateInterface::handleObjectScale] error scaling object!!");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleTemplateStatus(GetAffordanceTemplateStatus::Request &req, GetAffordanceTemplateStatus::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleTemplateStatus] getting status of templates...");

    if (!req.name.empty())
    {
        std::vector<std::string> keys;
        boost::split(keys, req.name, boost::is_any_of(":"));
        if (keys.size() >= 2)
        {
            int id = std::stoi(keys[1]);
            res.affordance_template_status.push_back(getTemplateStatus(keys[0], id, req.trajectory_name, req.frame_id));
            ATPointer at;
            if ( at_server_->getTemplateInstance(req.name, at))
            {
                res.current_trajectory = at->getCurrentTrajectory();
                AffordanceTemplateStructure ats = at->getCurrentStructure();
                for (auto t : ats.ee_trajectories)
                    res.trajectory_names.push_back(t.name);
            }
            else
                ROS_ERROR("[AffordanceTemplateInterface::handleTemplateStatus] %s not current running on server!!", req.name.c_str());
        }
        else
            ROS_ERROR("[AffordanceTemplateInterface::handleTemplateStatus] %s is an invalid template name!", req.name.c_str());
    }
    else
        ROS_ERROR("[AffordanceTemplateInterface::handleTemplateStatus] no template name provided!!");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleServerStatus(GetAffordanceTemplateServerStatus::Request &req, GetAffordanceTemplateServerStatus::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleServerStatus] getting server status...");
    
    res.ready = at_server_->getStatus();
    
    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleSetTrajectory(SetAffordanceTemplateTrajectory::Request &req, SetAffordanceTemplateTrajectory::Response &res)
{
    at_server_->setStatus(false);
    res.success = false;

    if (req.trajectory.empty())
        ROS_INFO("[AffordanceTemplateInterface::handleSetTrajectory] setting trajectory %s to current trajectory", req.name.c_str());
    else
        ROS_INFO("[AffordanceTemplateInterface::handleSetTrajectory] setting trajectory %s to %s", req.name.c_str(), req.trajectory.c_str());

    ATPointer at;
    if ( at_server_->getTemplateInstance(req.name, at))
    {
        if ( !at->switchTrajectory(req.name))
            res.success = true;
        else
            ROS_ERROR("[AffordanceTemplateInterface::handleSetTrajectory] error setting trajectory %s", req.trajectory.c_str());
    }
    else
        ROS_ERROR("[AffordanceTemplateInterface::handleSetTrajectory] %s template is not currently running on server!!", req.name.c_str());        
    
    at_server_->setStatus(true);
    return res.success;
}

bool AffordanceTemplateInterface::handleSetPose(SetAffordanceTemplatePose::Request &req, SetAffordanceTemplatePose::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleSetPose] setting pose for %s:%d", req.class_type.c_str(), req.id);

    res.success = at_server_->updateTemplate(req.class_type, req.id, req.pose);
    
    at_server_->setStatus(true);
    return true;
}


void AffordanceTemplateInterface::handleObjectScaleCallback(const ScaleDisplayObjectInfo &data)
{
    ROS_INFO_STREAM("[AffordanceTemplateInterface::handleObjectScaleCallback] scale "<<data.class_type<<":"<<data.id<<"->object["<<data.object_name<<"] by ("<<data.scale_factor<<", "<<data.end_effector_scale_factor<<")");

    ATPointer at;
    if ( at_server_->getTemplateInstance(data.class_type, data.id, at))
        if ( !at->scaleObject(data.object_name, data.scale_factor, data.end_effector_scale_factor))
            ROS_ERROR("[AffordanceTemplateInterface::handleObjectScaleCallback] error trying to scale object!!");
}

// @seth 10/28/2015 -- may not be complete??
// should be double checked by @swhart
AffordanceTemplateStatus AffordanceTemplateInterface::getTemplateStatus(const std::string& type, const int id, std::string& trajectory, const std::string& frame_id)
{
    AffordanceTemplateStatus ats; 

    ATPointer at;
    if ( !at_server_->getTemplateInstance(type, id, at))
    {
        ROS_ERROR("[AffordanceTemplateInterface::getTemplateStatus] %s:%d template is not currently running on server!!", type.c_str(), id);
        return ats;
    }

    ats.type = type;
    ats.id = id;
    if (trajectory.empty())
        trajectory = at->getCurrentTrajectory();
    ats.trajectory_name = trajectory;

    AffordanceTemplateStructure at_struct = at->getCurrentStructure();

    bool found = false;
    for ( auto traj : at_struct.ee_trajectories)
    {
        if (traj.name == trajectory)
        {
            found = true;
            break;
        }
    }
    if (!found)
    {
        ROS_WARN("[AffordanceTemplateInterface::getTemplateStatus] trajectory name %s not found in template", trajectory.c_str());
        return ats;
    }

    for ( auto obj : at_struct.display_objects)
    {
        ObjectInfo oi;
        oi.object_name = obj.name;
        // oi.object_pose -- may not be necessary according to @swhart 10/27/2015
        ats.object_info.push_back(oi);
    }

    std::map<std::string, int> ee_names = at->getRobotInterface()->getEEIDMap();
    for (auto ee : ee_names)
    {
        WaypointInfo wpi;
        wpi.end_effector_name = ee.first;
        wpi.id = ee.second;
        wpi.num_waypoints = at->getNumWaypoints(at_struct, at->getCurrentTrajectory(), wpi.id);

        affordance_template::PlanStatus ps;
        if (!at->getTrajectoryPlan(ats.trajectory_name, ee.first, ps))
        {
            ROS_WARN("[AffordanceTemplateInterface::getTemplateStatus] trajectory %s for end effector %s doesn't exist!!", ats.trajectory_name.c_str(), ee.first.c_str());
            continue;
        }
        wpi.waypoint_index = ps.current_idx; // is this reversed??
        wpi.plan_valid = ps.plan_valid;
        wpi.execution_valid = ps.exec_valid;
        wpi.waypoint_plan_index = ps.goal_idx; // is this reversed??
        for ( auto p : ps.sequence_poses)
            wpi.waypoint_poses.push_back(p);
        // the python had ee_pose_name data struct here -- don't know if it's necessary though

        ats.waypoint_info.push_back(wpi);   
    }

    return ats;
}
