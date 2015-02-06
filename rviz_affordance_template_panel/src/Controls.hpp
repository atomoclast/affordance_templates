#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include <ros/ros.h>

#include <iostream>
#include <boost/shared_ptr.hpp>

#include <QTableWidgetItem>

#include "RobotConfig.hpp"
#include "util.hpp"
#include "ui_rviz_affordance_template_panel.h"

#include <affordance_template_msgs/AffordanceTemplatePlanCommand.h>
#include <affordance_template_msgs/AffordanceTemplateExecuteCommand.h>
#include "AffordanceTemplateStatusInfo.hpp"

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
        Controls(Ui::RVizAffordanceTemplatePanel* ui);
        ~Controls() {};

        void setServices(ros::ServiceClient plan_srv, ros::ServiceClient execute_srv) { planService_ = plan_srv; executeService_ = execute_srv; };
        void setRobotMap(std::map<std::string, RobotConfigSharedPtr> map) { robotMap_ = map; };
        void setRobotName(std::string name) { robotName_ = name; };
        bool requestPlan(Controls::CommandType command_type);
        bool executePlan();
        void updateTable(std::map<int, std::pair<int,int> > waypointData);
        
    private:

        std::vector<std::string> getSelectedEndEffectors();
        std::vector<int> getSelectedEndEffectorWaypointIDs();
        std::vector<std::pair<std::string,int> > getSelectedEndEffectorInfo();

        Ui::RVizAffordanceTemplatePanel* ui_;
        ros::ServiceClient planService_;
        ros::ServiceClient executeService_;
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string robotName_;

    };
}

#endif // CONTROLS_HPP
