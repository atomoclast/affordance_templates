#include <affordance_template_server/interface.h>

using namespace affordance_template_server;
using namespace affordance_template_msgs;
using namespace affordance_template_object;

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
    at_srv_map_["get_status"]              = nh.advertiseService(base_srv + "get_status", &AffordanceTemplateInterface::handleServerStatus, this);
    at_srv_map_["get_template_status"]     = nh.advertiseService(base_srv + "get_template_status", &AffordanceTemplateInterface::handleTemplateStatus, this);
    at_srv_map_["set_template_trajectory"] = nh.advertiseService(base_srv + "set_template_trajectory", &AffordanceTemplateInterface::handleSetTrajectory, this);
    at_srv_map_["set_template_pose"]       = nh.advertiseService(base_srv + "set_template_pose", &AffordanceTemplateInterface::handleSetPose, this);
    at_srv_map_["set_object_pose"]         = nh.advertiseService(base_srv + "set_object_pose", &AffordanceTemplateInterface::handleSetObject, this);
    at_srv_map_["get_object_pose"]         = nh.advertiseService(base_srv + "get_object_pose", &AffordanceTemplateInterface::handleGetObject, this);
    at_srv_map_["set_waypoint_view"]       = nh.advertiseService(base_srv + "set_waypoint_view", &AffordanceTemplateInterface::handleSetWaypointViews, this);

    scale_stream_sub_ = nh.subscribe(base_srv + "scale_object_streamer", 1000, &AffordanceTemplateInterface::handleObjectScaleCallback, this);

    ROS_INFO("[AffordanceTemplateInterface] services set up...robot ready!");
}

bool AffordanceTemplateInterface::handleRobotRequest(GetRobotConfigInfo::Request &req, GetRobotConfigInfo::Response &res)
{
    at_server_->setStatus(false);
    
    ROS_INFO("[AffordanceTemplateInterface::handleRobotRequest] requesting robot configuration from server");
    res.robots.push_back(at_server_->getRobotConfig());
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

    if (!res.status)
        ROS_ERROR("[AffordanceTemplateInterface::handleAddTemplate] error adding template!!");
    else
        ROS_INFO("[AffordanceTemplateInterface::handleAddTemplate] added template: %s",(req.class_type+":"+std::to_string(res.id)).c_str());

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
    ROS_DEBUG("[AffordanceTemplateInterface::handleRunning] getting names of running templates");

    std::vector<std::string> templates = at_server_->getRunningTemplates();
    for (auto t : templates)
    {
        ROS_DEBUG("[AffordanceTemplateInterface::handleRunning] \tfound template: %s", t.c_str());
        res.templates.push_back(t);
    }

    if (templates.size() == 0)
        ROS_DEBUG("[AffordanceTemplateInterface::handleRunning] no templates are currently running on server");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handlePlanCommand(AffordanceTemplatePlanCommand::Request &req, AffordanceTemplatePlanCommand::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handlePlanCommand] new plan request for %s:%d, trajectory %s", req.type.c_str(), req.id, req.trajectory_name.c_str());

    ATPointer at;
    if (!at_server_->getTemplateInstance(req.type, req.id, at))
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
            if (!doesEndEffectorExist(at, ee))
                continue;
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

    ATPointer at;
    if (!at_server_->getTemplateInstance(req.type, req.id, at))
        ROS_ERROR("[AffordanceTemplateInterface::handleExecuteCommand] error getting instance of affordance template %s:%d", req.type.c_str(), req.id);
    else
    {
        // check if specific trajectory was given
        if (req.trajectory_name.empty())
            req.trajectory_name = at->getCurrentTrajectory();

        std::vector<std::string> ee_list;
        for (auto ee : req.end_effectors)
        {
            if (!doesEndEffectorExist(at, ee))
                ROS_WARN("[AffordanceTemplateInterface::handleExecuteCommand] %s not in trajectory, can't execute!!", ee.c_str());
            else
                ee_list.push_back(ee);
        }

        res.status = at->moveToWaypoints(ee_list);
        
        if (res.status)
            ROS_INFO("[AffordanceTemplateInterface::handleExecuteCommand] done executing %s plan(s)", req.trajectory_name.c_str());
        else
            ROS_ERROR("[AffordanceTemplateInterface::handleExecuteCommand] execution failed for %s", req.trajectory_name.c_str());
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
    if (at_server_->getTemplateInstance(req.original_class_type, req.id, at))
        save_status = at->saveToDisk(req.filename, req.image, new_key, req.save_scale_updates);
    if (!at_server_->refreshTemplates())
        ROS_ERROR("[AffordanceTemplateInterface::handleSaveTemplate] failed to refresh templates. unexpected behavior possible!!");
    bool remove_status = at_server_->removeTemplate(req.original_class_type, req.id);
    bool add_status = at_server_->addTemplate(req.new_class_type, req.id);
    res.status = save_status && remove_status && add_status;
    if (!res.status)
        ROS_ERROR("[AffordanceTemplateInterface::handleSaveTemplate] error saving template. save to file: %s, remove: %s, adding: %s", successToString(save_status).c_str(), successToString(remove_status).c_str(), successToString(add_status).c_str());

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleAddTrajectory(AddAffordanceTemplateTrajectory::Request &req, AddAffordanceTemplateTrajectory::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleAddTrajectory] adding new trajectory \'%s\' to template %s:%d", req.trajectory_name.c_str(), req.class_type.c_str(), req.id);

    res.status = false;

    ATPointer at;
    if (at_server_->getTemplateInstance(req.class_type, req.id, at))
        res.status = at->addTrajectory(req.trajectory_name);
    else ROS_WARN("[AffordanceTemplateInterface::handleAddTrajectory] error getting instance of AT");

    if (!res.status)
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
    if (at_server_->getTemplateInstance(req.scale_info.class_type, req.scale_info.id, at))
        res.status = at->setObjectScaling(req.scale_info.object_name, req.scale_info.scale_factor, req.scale_info.end_effector_scale_factor);

    if (!res.status)
        ROS_ERROR("[AffordanceTemplateInterface::handleObjectScale] error scaling object!!");

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleTemplateStatus(GetAffordanceTemplateStatus::Request &req, GetAffordanceTemplateStatus::Response &res)
{
    at_server_->setStatus(false);
    ROS_DEBUG("[AffordanceTemplateInterface::handleTemplateStatus] getting status of templates...");

    if (!req.name.empty())
    {
        std::vector<std::string> keys;
        boost::split(keys, req.name, boost::is_any_of(":"));
        if (keys.size() >= 2)
        {
            int id = std::stoi(keys[1]);
            res.affordance_template_status.push_back(getTemplateStatus(keys[0], id, req.trajectory_name, req.frame_id));
            ATPointer at;
            if (at_server_->getTemplateInstance(req.name, at))
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
    res.ready = at_server_->getStatus();

    ROS_DEBUG_STREAM("[AffordanceTemplateInterface::handleServerStatus] getting server status..."<<boolToString(res.ready));

    return true;
}

bool AffordanceTemplateInterface::handleSetTrajectory(SetAffordanceTemplateTrajectory::Request &req, SetAffordanceTemplateTrajectory::Response &res)
{
    res.success = false;
    if (req.name.empty())
    {
        ROS_WARN("[AffordanceTemplateInterface::handleSetTrajectory] no template provided, ignoring.");
        return true;
    }

    if (req.trajectory.empty())
        ROS_INFO("[AffordanceTemplateInterface::handleSetTrajectory] setting trajectory %s to current trajectory", req.name.c_str());
    else
        ROS_INFO("[AffordanceTemplateInterface::handleSetTrajectory] setting trajectory %s to %s", req.name.c_str(), req.trajectory.c_str());

    ATPointer at;
    if (at_server_->getTemplateInstance(req.name, at))
    {
        if (at->setTrajectory(req.trajectory))
            res.success = true;
        else
            ROS_ERROR("[AffordanceTemplateInterface::handleSetTrajectory] error setting trajectory %s", req.trajectory.c_str());
    }
    else
        ROS_ERROR("[AffordanceTemplateInterface::handleSetTrajectory] %s template is not currently running on server!!", req.name.c_str());        
    
    return true;
}

bool AffordanceTemplateInterface::handleSetPose(SetAffordanceTemplatePose::Request &req, SetAffordanceTemplatePose::Response &res)
{
    ROS_INFO("[AffordanceTemplateInterface::handleSetPose] setting pose for %s:%d", req.class_type.c_str(), req.id);

    res.success = at_server_->updateTemplate(req.class_type, req.id, req.pose);
    
    return true;
}

bool AffordanceTemplateInterface::handleSetObject(SetObjectPose::Request& req, SetObjectPose::Response& res)
{
    res.status = true;

    for (auto& o : req.objects)
    {
        ATPointer at;
        if (at_server_->getTemplateInstance(o.type, o.id, at))
        {
            ROS_WARN("[AffordanceTemplateInterface::handleSetObject] will call AT %s:%d method to set pose for object %s", o.type.c_str(), o.id, o.name.c_str());
            if (!at->setObjectPose(o))
            {
                ROS_ERROR("[AffordanceTemplateInterface::handleSetObject] failed to set pose for object %s most likely couldn't find object in structure", o.name.c_str());
                res.status = false;
                break;
            }
        }
        else
        {
            ROS_ERROR("[AffordanceTemplateInterface::handleSetObject] failed to find template %s:%d -- won't be able to set pose for object %s", o.type.c_str(), o.id, o.name.c_str());
            res.status = false;
        }
    }

    return true;
}

bool AffordanceTemplateInterface::handleGetObject(GetObjectPose::Request& req, GetObjectPose::Response& res)
{
    ATPointer at;
    if (at_server_->getTemplateInstance(req.name, at))
    {
        affordance_template_object::AffordanceTemplateStructure ats = at->getCurrentStructure();
        ROS_INFO("[AffordanceTemplateInterface::handleGetObject] getting pose for object %s", req.name.c_str());
        for (auto d : ats.display_objects)
        {
            ObjectInfo obj; 
            obj.object_name = d.name;
            geometry_msgs::PoseStamped ps;
            if (d.parent.empty())
                ps.header.frame_id = d.name;
            else
                ps.header.frame_id = d.parent;
            ps.pose = affordance_template_object::originToPoseMsg(d.origin);
            obj.object_pose = ps;

            if (req.name.empty())
                res.objects.push_back(obj);
            else if (d.name == req.name)
            {
                res.objects.push_back(obj);
                break;
            }
        }
    }

    return true;
}

bool AffordanceTemplateInterface::handleSetWaypointViews(SetWaypointViewModes::Request& req, SetWaypointViewModes::Response& res)
{

    ROS_DEBUG("[AffordanceTemplateInterface::handleSetWaypointViews] setting waypoint view modes...");

    std::vector<std::string> at_keys, wp_keys;
    int at_id, wp_id, ee_id, idx;
    std::string at_class;

    idx=0;
    if (!req.waypoint_names.empty()) {
      if(req.waypoint_names.size() != req.compact_view.size()) {
        ROS_ERROR("[AffordanceTemplateInterface::handleSetWaypointViews] size mismatch between names and vals");
        return false;
      }
      for(auto &wp : req.waypoint_names) {
        boost::split(at_keys, wp, boost::is_any_of(":"));
        if (at_keys.size() == 3) {
          at_class = at_keys[1];
          at_id = std::stoi(at_keys[2]);
          boost::split(wp_keys, at_keys[0], boost::is_any_of("."));
          if (wp_keys.size() == 2) {
            ee_id = std::stoi(wp_keys[0]);
            wp_id = std::stoi(wp_keys[1]);
            //std::cout << "waypoint[" << idx << "]: " << wp << std::endl; 
            // std::cout << "  AT Type: " << at_class << std::endl; 
            // std::cout << "  AT ID:   " << at_id << std::endl; 
            // std::cout << "  EE ID:   " << ee_id << std::endl; 
            // std::cout << "  WP ID:   " << wp_id << std::endl; 
            //std::cout << "  mode:   " << (int)req.compact_view[idx] << std::endl; 

            ATPointer at;
            if (at_server_->getTemplateInstance(at_class, at_id, at)) {
              at->setWaypointViewMode(ee_id, wp_id, req.compact_view[idx]);
            }

          } else {
            ROS_ERROR("[AffordanceTemplateInterface::handleSetWaypointViews] error parsing wp details");
            return false;
          }  
        } else {
          ROS_ERROR("[AffordanceTemplateInterface::handleSetWaypointViews] error parsing wp");
          return false;
        }
        idx++;
      }
    } else {
      ROS_INFO("[AffordanceTemplateInterface::handleSetWaypointViews] setting mode for all waypoints");
    }
    return true;
}

void AffordanceTemplateInterface::handleObjectScaleCallback(const ScaleDisplayObjectInfo &data)
{
    ROS_DEBUG("[AffordanceTemplateInterface::handleObjectScaleCallback] scale %s:%d->object[%s] by: %g, %g", data.class_type.c_str(), data.id, data.object_name.c_str(), data.scale_factor, data.end_effector_scale_factor);

    std::string key = data.object_name + ":" + std::to_string(data.id);

    ATPointer at;
    if (at_server_->getTemplateInstance(data.class_type, data.id, at))
        if (!at->setObjectScaling(key, data.scale_factor, data.end_effector_scale_factor))
            ROS_ERROR("[AffordanceTemplateInterface::handleObjectScaleCallback] error trying to scale object!!");
}




// @seth 10/28/2015 -- may not be complete??
// should be double checked by @swhart
AffordanceTemplateStatus AffordanceTemplateInterface::getTemplateStatus(const std::string& type, const int id, std::string& trajectory, const std::string& frame_id)
{
    AffordanceTemplateStatus ats; 

    ATPointer at;
    if (!at_server_->getTemplateInstance(type, id, at))
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
    affordance_template::WaypointTrajectoryFlags wp_flags; 
    if (!doesTrajectoryExist(at, ats.trajectory_name))
        return ats;

    bool has_flags = at->getWaypointFlags(ats.trajectory_name, wp_flags);

    for (auto obj : at_struct.display_objects)
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

        ROS_DEBUG("[AffordanceTemplateInterface::getTemplateStatus] parsing ee -- %s, id -- %d", ee.first.c_str(), wpi.id);

        wpi.num_waypoints = at->getNumWaypoints(at_struct, at->getCurrentTrajectory(), wpi.id);
                
        affordance_template::PlanStatus ps;
        if (at->getTrajectoryPlan(ats.trajectory_name, ee.first, ps)) {
            ROS_WARN("[AffordanceTemplateInterface::getTemplateStatus] trajectory %s for end effector %s doesn't have a valid plan!!", ats.trajectory_name.c_str(), ee.first.c_str());
            wpi.waypoint_index = ps.current_idx;
            wpi.plan_valid = ps.plan_valid;
            wpi.execution_valid = ps.exec_valid;
            wpi.waypoint_plan_index = ps.goal_idx;
            for (auto p : ps.sequence_poses)
                wpi.waypoint_poses.push_back(p);
        } else {
            wpi.waypoint_index = -1; 
            wpi.plan_valid = false;
            wpi.execution_valid = false;
            wpi.waypoint_plan_index = -1;
        }

        // set compact and controls display flags
        if(has_flags) {
            for(int idx=0; idx<wpi.num_waypoints; idx++) {
                std::string wp_name = std::to_string(wpi.id) + "." + std::to_string(idx) + ":" + ats.type + ":" + std::to_string(ats.id); 
                wpi.compact_view.push_back(wp_flags.compact_view[wp_name]);
            }
        }
        ats.waypoint_info.push_back(wpi);   
    }

    return ats;
}

bool AffordanceTemplateInterface::doesTrajectoryExist(const ATPointer& atp, const std::string& trajectory)
{
    bool found = false;
    
    AffordanceTemplateStructure ats = atp->getCurrentStructure();
    for (auto traj : ats.ee_trajectories)
    {
        if (traj.name == trajectory)
        {
            found = true;
            break;
        }
    }

    if (!found)
        ROS_WARN("[AffordanceTemplateInterface::doesTrajectoryExist] trajectory name %s not found in template", trajectory.c_str());

    return found;
}

bool AffordanceTemplateInterface::doesEndEffectorExist(const ATPointer& atp, const std::string& ee)
{
    bool found = false;

    std::map<int, std::string> ee_map = atp->getRobotInterface()->getEENameMap();
    for (auto e : ee_map)
    {
        if (e.second == ee)
        {
            found = true;
            break;
        }
    }

    return found;
}