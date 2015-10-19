#include <affordance_template_server/interface.h>

using namespace affordance_template_server;
using namespace affordance_template_msgs;

AffordanceTemplateInterface::AffordanceTemplateInterface()
{

}

AffordanceTemplateInterface::~AffordanceTemplateInterface(){}

bool handleRobotRequest(GetRobotConfigInfo::Request &req, GetRobotConfigInfo::Response &res)
{
    return true;
}

bool handleTemplateRequest(GetAffordanceTemplateConfigInfo::Request &req, GetAffordanceTemplateConfigInfo::Response &res)
{
    return true;
}

bool handleLoadRobot(LoadRobotConfig::Request &req, LoadRobotConfig::Response &res)
{
    return true;
}

bool handleAddTemplate(AddAffordanceTemplate::Request &req, AddAffordanceTemplate::Response &res)
{
    return true;
}

bool handleRunning(GetRunningAffordanceTemplates::Request &req, GetRunningAffordanceTemplates::Response &res)
{
    return true;
}

bool handleTemplateKill(DeleteAffordanceTemplate::Request &req, DeleteAffordanceTemplate::Response &res)
{
    return true;
}

bool handlePlanCommand(AffordanceTemplatePlanCommand::Request &req, AffordanceTemplatePlanCommand::Response &res)
{
    return true;
}

bool handleExecuteCommand(AffordanceTemplateExecuteCommand::Request &req, AffordanceTemplateExecuteCommand::Response &res)
{
    return true;
}

bool handleSaveTemplate(SaveAffordanceTemplate::Request &req, SaveAffordanceTemplate::Response &res)
{
    return true;
}

bool handleAddTrajectory(AddAffordanceTemplateTrajectory::Request &req, AddAffordanceTemplateTrajectory::Response &res)
{
    return true;
}

bool handleObjectScale(ScaleDisplayObject::Request &req, ScaleDisplayObject::Response &res)
{
    return true;
}

bool handleTemplateStatusRequest(GetAffordanceTemplateStatus::Request &req, GetAffordanceTemplateStatus::Response &res)
{
    return true;
}

bool handleServerStatusRequest(GetAffordanceTemplateServerStatus::Request &req, GetAffordanceTemplateServerStatus::Response &res)
{
    return true;
}

bool handleSetTrajectory(SetAffordanceTemplateTrajectory::Request &req, SetAffordanceTemplateTrajectory::Response &res)
{
    return true;
}

bool handleSetPose(SetAffordanceTemplatePose::Request &req, SetAffordanceTemplatePose::Response &res)
{
    return true;
}


bool handleObjectScaleCallback(const ScaleDisplayObjectInfo &data)
{
    return true;
}

bool getTemplateStatus(AffordanceTemplateStatus &status, std::string template_name, int template_id, std::string traj_name, std::string frame_id="")
{
    return true;
}
