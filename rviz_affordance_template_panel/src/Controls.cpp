#include "Controls.hpp"

using namespace rviz_affordance_template_panel;
using namespace std;

Controls::Controls(Ui::RVizAffordanceTemplatePanel* ui) :
    ui_(ui) {}

void Controls::updateTable(std::map<int, std::pair<int,int> > waypointData) {
    for (auto& wp : waypointData) {
        for (auto& e: (*robotMap_[robotName_]).endeffectorMap) {
            if (e.second->id() != wp.first) {
                continue;
            }
            for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
                if (e.second->name() != ui_->end_effector_table->item(r,0)->text().toStdString()) {
                    continue;
                }
                QTableWidgetItem* item_idx = ui_->end_effector_table->item(r, 1);
                item_idx->setText(QString::number(wp.second.first));

                QTableWidgetItem* item_n = ui_->end_effector_table->item(r, 2);
                item_n->setText(QString::number(wp.second.second));
            }
        }
    }
}

void Controls::requestPlan(Controls::CommandType command_type) {

    // need to determine where each waypoint currently is (get this from stored information)
    // the request needs:
    
    /*
    # command types
    uint8 PLAN = 0
    uint8 EXECUTE = 1

    # the template type
    string type

    # the template id
    uint8 id

    # which command type (as above)
    uint8 command

    # which end effectors to plan/execute
    string[] end_effectors

    # the goal IDs for each end-effector
    uint8 waypoint_goal_ids[]

    # go directly to the waypoint (if not adjacent)
    bool direct
    */

    string key = ui_->control_template_box->currentText().toUtf8().constData();
    vector<string> stuff = util::split(key, ':');
    map< int, pair<int,int> > waypointData;

    ROS_INFO("Sending Command request for a %s", key.c_str());      

    affordance_template_msgs::AffordanceTemplateCommand ;
    srv.request.type = stuff[0];
    srv.request.id = int(atoi(stuff[1].c_str()));
    srv.request.command = srv.request.PLAN;    
    
    //srv.request.command = command_type; 
    //srv.request.steps = ui_->num_steps->text().toInt();
    //srv.request.execute_on_plan = ui_->execute_on_plan->isChecked();
    //srv.request.execute_precomputed_plan = false;

    if(command_type==CommandType::START || command_type==CommandType::END) {
        srv.request.direct = true;
    }

    vector<string> ee_list = getSelectedEndEffectors();
    for(auto &ee : ee_list) {
        srv.request.end_effectors.push_back(ee);
        srv.request.waypoint_goal_ids.push_back(goal_id); ##### FIXME
    }

    if (controlsService_.call(srv))
    {
        ROS_INFO("Command successful");
        for(auto &wp : srv.response.waypoint_info) {
            pair<int,int> waypointPair;
            waypointPair = make_pair(int(wp.waypoint_index), int(wp.num_waypoints));
            waypointData[int(wp.id)] = waypointPair;
        }
        updateTable(waypointData);
        return srv.response.status;
    }
    else
    {
        ROS_ERROR("Failed to call service command");
        return false;
    }

    return false;
}


void Controls::executePlan() {

    // This all needs to be fixed

    string key = ui_->control_template_box->currentText().toUtf8().constData();
    vector<string> stuff = util::split(key, ':');
    map< int, pair<int,int> > waypointData;

    ROS_INFO("Sending Execute Command request for a %s", key.c_str());      

    affordance_template_msgs::AffordanceTemplateCommand srv;
    srv.request.type = stuff[0];
    srv.request.id = int(atoi(stuff[1].c_str()));
/*  
    srv.request.command = command_type;
    srv.request.steps = ui_->num_steps->text().toInt();
    srv.request.execute_on_plan = ui_->execute_on_plan->isChecked();
    srv.request.execute_precomputed_plan = true;
*/
    srv.request.command = srv.request.EXECUTE;    
    
    vector<string> ee_list = getSelectedEndEffectors();
    for(auto &ee : ee_list) {
        srv.request.end_effectors.push_back(ee);
    }

    if (controlsService_.call(srv))
    {
        ROS_INFO("Command successful");
        for(auto &wp : srv.response.waypoint_info) {
            pair<int,int> waypointPair;
            waypointPair = make_pair(int(wp.waypoint_index), int(wp.num_waypoints));
            waypointData[int(wp.id)] = waypointPair;
        }
        updateTable(waypointData);
        return srv.response.status;
    }
    else
    {
        ROS_ERROR("Failed to call service command");
        return false;
    }

    return false;
}


vector<string> Controls::getSelectedEndEffectors() {
    vector<string> selectedEndEffectors;
    for (int r=0; r<ui_->end_effector_table->rowCount(); r++ ) {
        if (ui_->end_effector_table->item(r,3)->checkState() == Qt::Checked ) {
            selectedEndEffectors.push_back(ui_->end_effector_table->item(r,0)->text().toStdString());
        }
    }
    return selectedEndEffectors;
}
