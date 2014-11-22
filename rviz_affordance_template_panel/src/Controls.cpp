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

void Controls::sendCommand(int command_type) {


    string key = ui_->control_template_box->currentText().toUtf8().constData();
    vector<string> stuff = util::split(key, ':');
    map< int, pair<int,int> > waypointData;

    ROS_INFO("Sending Command request for a %s", key.c_str());      

    affordance_template_msgs::AffordanceTemplateCommand srv;
    srv.request.type = stuff[0];
    srv.request.id = int(atoi(stuff[1].c_str()));
    srv.request.command = command_type;
    srv.request.steps = ui_->num_steps->text().toInt();
    srv.request.execute_on_plan = ui_->execute_on_plan->isChecked();
    srv.request.execute_precomputed_plan = true;

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
    }
    else
    {
        ROS_ERROR("Failed to call service command");
    }
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
