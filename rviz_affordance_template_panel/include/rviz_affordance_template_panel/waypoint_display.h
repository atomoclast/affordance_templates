#ifndef WAYPOINT_DISPLAY_HPP
#define WAYPOINT_DISPLAY_HPP

#include <ros/ros.h>

#include <iostream>
#include <boost/shared_ptr.hpp>

#include <QTableWidgetItem>
#include <QTreeWidgetItem>

#include <rviz_affordance_template_panel/robot_config.h>
#include <rviz_affordance_template_panel/util.h>
#include <rviz_affordance_template_panel/template_status_info.h>

#include "ui_rviz_affordance_template_panel.h"

#include <affordance_template_msgs/SetWaypointViewModes.h>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class WaypointDisplay : public QObject
    {

    Q_OBJECT
    public:

        typedef boost::shared_ptr<RobotConfig> RobotConfigSharedPtr;

        WaypointDisplay(QObject *_parent=0);
        ~WaypointDisplay() {};

        void setService(ros::ServiceClient set_srv) { setWaypointDisplayService_ = set_srv; };
        void setRobotMap(std::map<std::string, RobotConfigSharedPtr> map) { robotMap_ = map; };
        void setRobotName(std::string name) { robotName_ = name; };
        void setupWaypointDisplayInfo(AffordanceTemplateStatusInfo::EndEffectorInfo wp_info);

        void setUI(Ui::RVizAffordanceTemplatePanel* ui) { 
            ui_ = ui;
            QObject::connect(ui_->waypointDisplayTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(displayEventChecked(QTreeWidgetItem*, int)), Qt::UniqueConnection);
         }

        void setTemplateStatusInfo(AffordanceTemplateStatusInfo *template_status) { template_status_ = template_status; }
        AffordanceTemplateStatusInfo * getTemplateStatusInfo() { return template_status_; }

        void callWaypointDisplayService(std::vector<std::string> wp_names, std::vector<bool> modes);

    public Q_SLOTS:
        void displayEventChecked(QTreeWidgetItem *item,int i);

    private:

        Ui::RVizAffordanceTemplatePanel* ui_;
    
        ros::ServiceClient setWaypointDisplayService_;
    
        std::map<std::string, RobotConfigSharedPtr> robotMap_;
        std::string robotName_;
        AffordanceTemplateStatusInfo *template_status_;

        std::map<std::pair<std::string,int>, QTreeWidgetItem*> waypointDisplayItems_;
        std::map<std::string,int> eeNameMap_;

        std::map<std::string,bool> expandStatus_;

    };
}

#endif // WAYPOINT_DISPLAY_HPP