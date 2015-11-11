#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include <ros/ros.h>

#include <iostream>
#include <boost/shared_ptr.hpp>

#include <QTableWidgetItem>

#include <rviz_affordance_template_panel/robot_config.h>
#include <rviz_affordance_template_panel/util.h>
#include <rviz_affordance_template_panel/template_status_info.h>

#include "ui_rviz_affordance_template_panel.h"

#include <affordance_template_msgs/AffordanceTemplatePlanCommand.h>
#include <affordance_template_msgs/AffordanceTemplateExecuteCommand.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class Controls
    {
    public:

        enum CommandType 
        {
            START,
            END,
            STEP_FORWARD,
            STEP_BACKWARD,
            CURRENT,
            PAUSE
        };

        typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;

        Controls();
        ~Controls() {};

        void setServices(ros::ServiceClient plan_srv, ros::ServiceClient execute_srv) { planService_ = plan_srv; executeService_ = execute_srv; };
        void setRobotMap(std::map<std::string, RobotConfigSharedPtr> map) { robotMap_ = map; };
        void setRobotName(std::string name) { robotName_ = name; };
        void setTemplateStatusInfo(AffordanceTemplateStatusInfo *template_status) { template_status_ = template_status; }
        bool requestPlan(Controls::CommandType command_type);
        bool executePlan();
        void setUI(Ui::RVizAffordanceTemplatePanel* ui) { ui_ = ui; }

        AffordanceTemplateStatusInfo * getTemplateStatusInfo() { return template_status_; }
        std::vector<std::pair<std::string,int> > getSelectedEndEffectorInfo();

    private:

        Ui::RVizAffordanceTemplatePanel* ui_;
    
        ros::ServiceClient planService_;
        ros::ServiceClient executeService_;
    
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string robotName_;
        AffordanceTemplateStatusInfo *template_status_;

    };
}

#endif // CONTROLS_HPP