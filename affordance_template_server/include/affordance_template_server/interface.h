#ifndef _AFFORDANCE_TEMPLATE_INTERFACE_H_
#define _AFFORDANCE_TEMPLATE_INTERFACE_H_

#include <affordance_template_server/server.h>
#include <affordance_template_library/affordance_template_structure.h>

#include <affordance_template_msgs/GetRobotConfigInfo.h>
#include <affordance_template_msgs/GetAffordanceTemplateConfigInfo.h>
#include <affordance_template_msgs/LoadRobotConfig.h>
#include <affordance_template_msgs/AddAffordanceTemplate.h>
#include <affordance_template_msgs/DeleteAffordanceTemplate.h>
#include <affordance_template_msgs/GetRunningAffordanceTemplates.h>
#include <affordance_template_msgs/AffordanceTemplatePlanCommand.h>
#include <affordance_template_msgs/AffordanceTemplateExecuteCommand.h>
#include <affordance_template_msgs/SaveAffordanceTemplate.h>
#include <affordance_template_msgs/AddAffordanceTemplateTrajectory.h>
#include <affordance_template_msgs/ScaleDisplayObject.h>
#include <affordance_template_msgs/ScaleDisplayObjectInfo.h>
#include <affordance_template_msgs/GetAffordanceTemplateStatus.h>
#include <affordance_template_msgs/GetAffordanceTemplateServerStatus.h>
#include <affordance_template_msgs/SetAffordanceTemplateTrajectory.h>
#include <affordance_template_msgs/SetAffordanceTemplatePose.h>
#include <affordance_template_msgs/ObjectInfo.h>
#include <affordance_template_msgs/WaypointInfo.h>

using namespace affordance_template_msgs;

namespace affordance_template_server
{
    typedef boost::shared_ptr<affordance_template::AffordanceTemplate> ATPointer;

    inline std::string boolToString(bool b) { return (b ? "true" : "false"); }
    inline std::string successToString(bool b) { return (b ? "succeeded" : "failed"); }

    class AffordanceTemplateInterface
    {
        // srv handlers
        bool handleRobotRequest(GetRobotConfigInfo::Request&, GetRobotConfigInfo::Response&);
        bool handleTemplateRequest(GetAffordanceTemplateConfigInfo::Request&, GetAffordanceTemplateConfigInfo::Response&);
        bool handleLoadRobot(LoadRobotConfig::Request&, LoadRobotConfig::Response&);
        bool handleAddTemplate(AddAffordanceTemplate::Request&, AddAffordanceTemplate::Response&);
        bool handleDeleteTemplate(DeleteAffordanceTemplate::Request&, DeleteAffordanceTemplate::Response&);
        bool handleRunning(GetRunningAffordanceTemplates::Request&, GetRunningAffordanceTemplates::Response&);
        bool handlePlanCommand(AffordanceTemplatePlanCommand::Request&, AffordanceTemplatePlanCommand::Response&);
        bool handleExecuteCommand(AffordanceTemplateExecuteCommand::Request&, AffordanceTemplateExecuteCommand::Response&);
        bool handleSaveTemplate(SaveAffordanceTemplate::Request&, SaveAffordanceTemplate::Response&);
        bool handleAddTrajectory(AddAffordanceTemplateTrajectory::Request&, AddAffordanceTemplateTrajectory::Response&);
        bool handleObjectScale(ScaleDisplayObject::Request&, ScaleDisplayObject::Response&);
        bool handleTemplateStatus(GetAffordanceTemplateStatus::Request&, GetAffordanceTemplateStatus::Response&);
        bool handleServerStatus(GetAffordanceTemplateServerStatus::Request&, GetAffordanceTemplateServerStatus::Response&);
        bool handleSetTrajectory(SetAffordanceTemplateTrajectory::Request&, SetAffordanceTemplateTrajectory::Response&);
        bool handleSetPose(SetAffordanceTemplatePose::Request&, SetAffordanceTemplatePose::Response&);
        
        void handleObjectScaleCallback(const ScaleDisplayObjectInfo&);
        bool doesTrajectoryExist(const ATPointer&, const std::string&);
        bool doesEndEffectorExist(const ATPointer&, const std::string&);
        AffordanceTemplateStatus getTemplateStatus(const std::string& template_name, const int template_id, std::string& traj_name, const std::string& frame_id="");
        
        ros::Subscriber scale_stream_sub_;
        boost::shared_ptr<AffordanceTemplateServer> at_server_;
        tf::TransformListener listener_;
        std::map<std::string, ros::ServiceServer> at_srv_map_;

    public:
        AffordanceTemplateInterface(const std::string&);
        ~AffordanceTemplateInterface() {}
    };       
}

#endif