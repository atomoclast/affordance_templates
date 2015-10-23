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
    res.robots.push_back( at_server_->getRobotConfig() );
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
        templates = at_server_->getTemplate(req.name);
        if (templates.size())
            res.templates.push_back(templates.front());
        else
            ROS_WARN("[AffordanceTemplateInterface::handleTemplateRequest] couldn't find template on server matching template name: %s on server!!", req.name.c_str());
    }
    else
    {
        ROS_INFO("[AffordanceTemplateInterface::handleTemplateRequest] requesting infor for all loaded templates");
        templates = at_server_->getTemplate();
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

    std::vector<AffordanceTemplateConfig> templates = at_server_->getTemplate();
    for (auto t : templates)
    {
        ROS_INFO("[AffordanceTemplateInterface::handleRunning] found template: %s", t.type.c_str());
        res.templates.push_back(t.type);
    }

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handlePlanCommand(AffordanceTemplatePlanCommand::Request &req, AffordanceTemplatePlanCommand::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handlePlanCommand] new plan request for %s:%d, trajectory %s", req.type.c_str(), req.id, req.trajectory_name.c_str());

    boost::shared_ptr<affordance_template::AffordanceTemplate> at;
    if ( !at_server_->getTemplateInstance(req.type, req.id, at) )
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
        std::map<std::string, bool> req_map;
        for (auto ee : req.end_effectors)
        {

            steps = req.steps[id];
            // TODO
            // make sure EE is in trajectory
            // if not at->trajectory_has_ee(request.trajectory_name, ee): 
            //         rospy.logwarn(str("ServiceInterface::handle_plan_command() -- " + ee + " not in trajectory, can't plan"))
            //         continue
            ee_names.push_back(ee);
            ++id;
        }
    }

    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleExecuteCommand(AffordanceTemplateExecuteCommand::Request &req, AffordanceTemplateExecuteCommand::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleExecuteCommand]");
    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleSaveTemplate(SaveAffordanceTemplate::Request &req, SaveAffordanceTemplate::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleSaveTemplate]");
    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleAddTrajectory(AddAffordanceTemplateTrajectory::Request &req, AddAffordanceTemplateTrajectory::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleAddTrajectory]");
    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleObjectScale(ScaleDisplayObject::Request &req, ScaleDisplayObject::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleObjectScale]");
    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleTemplateStatus(GetAffordanceTemplateStatus::Request &req, GetAffordanceTemplateStatus::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleTemplateStatus]");
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
    ROS_INFO("[AffordanceTemplateInterface::handleSetTrajectory]");
    at_server_->setStatus(true);
    return true;
}

bool AffordanceTemplateInterface::handleSetPose(SetAffordanceTemplatePose::Request &req, SetAffordanceTemplatePose::Response &res)
{
    at_server_->setStatus(false);
    ROS_INFO("[AffordanceTemplateInterface::handleSetPose]");
    at_server_->setStatus(true);
    return true;
}


void AffordanceTemplateInterface::handleObjectScaleCallback(const ScaleDisplayObjectInfo &data)
{
    ROS_INFO_STREAM("[AffordanceTemplateInterface::handleObjectScaleCallback] scale "<<data.class_type<<":"<<data.id<<"->object["<<data.object_name<<"] by ("<<data.scale_factor<<", "<<data.end_effector_scale_factor<<")");
    try
    {
        // TODO
        // key = str(data.class_type) + ":" + str(data.id)
        // self.server.at_data.class_map[data.class_type][data.id].scale_object(data.object_name, data.scale_factor, data.end_effector_scale_factor)
    }
    catch(...)
    {
        ROS_ERROR("[AffordanceTemplateInterface::handleObjectScaleCallback] error trying to scale object!!");
    }
}

bool AffordanceTemplateInterface::getTemplateStatus(AffordanceTemplateStatus &status, std::string template_name, int template_id, std::string traj_name, std::string frame_id="")
{
    return true;
}
