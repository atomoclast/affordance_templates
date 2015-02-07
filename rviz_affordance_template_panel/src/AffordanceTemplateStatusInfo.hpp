#ifndef AFFORDANCE_TEMPLATE_STATUS_INFO_HPP
#define AFFORDANCE_TEMPLATE_STATUS_INFO_HPP

#include <affordance_template_msgs/AffordanceTemplateStatus.h>
#include <map>
#include <vector>
#include <string>

namespace Ui {
class RVizAffordanceTemplatePanel;
}

namespace rviz_affordance_template_panel
{
    class AffordanceTemplateStatusInfo
    {

	public:

    	typedef std::map<std::string, affordance_template_msgs::WaypointInfo*>  EndEffectorInfo;
    	typedef std::map<std::string, EndEffectorInfo> TrajectoryWaypointInfo;

		AffordanceTemplateStatusInfo(std::string name, int id) :
			name_(name),
			id_(id) {
		}
		~AffordanceTemplateStatusInfo() {}

		std::string getName() { return name_; }
		int getID() { return id_; }

		std::string getCurrentTrajectory() { return current_trajectory_; }
		void setCurrentTrajectory(std::string traj) { current_trajectory_ = traj; }
		
		TrajectoryWaypointInfo getTrajectoryInfo() { return trajectory_info_; }

		EndEffectorInfo getTrajectoryStatus(std::string traj_name) { return trajectory_info_[traj_name]; }

		bool updateTrajectoryStatus(const affordance_template_msgs::AffordanceTemplateStatusConstPtr& status) {

			if(status->type!=name_) {
				ROS_ERROR("AffordanceTemplateStatusInfo::addTrajectoryStatus() -- name mismatch: %s != %s", status->type.c_str(), name_.c_str());
				return false;
			}
			if(status->id!=id_) {
				ROS_ERROR("AffordanceTemplateStatusInfo::addTrajectoryStatus() -- ID mismatch: %d != %d", status->id, id_);
				return false;
			}
			if(status->trajectory_name=="") {
				ROS_ERROR("AffordanceTemplateStatusInfo::addTrajectoryStatus() -- no trajectory name given");
				return false;
			}

			for (auto &w : status->waypoint_info) {
				
				std::string ee =  w.end_effector_name;

				if(trajectory_info_[status->trajectory_name].find(ee) == trajectory_info_[status->trajectory_name].end()) {
					trajectory_info_[status->trajectory_name][ee] = new affordance_template_msgs::WaypointInfo();
				}

				ROS_INFO("AffordanceTemplateStatusInfo::updateTrajectoryStatus() -- adding waypoint info for ee: %s", ee.c_str());
				ROS_INFO("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- id: %d", w.id);
				ROS_INFO("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- num_waypoints: %d", (int)(w.num_waypoints));
				ROS_INFO("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- waypoint_index: %d", (int)(w.waypoint_index));
				ROS_INFO("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- waypoint_plan_index: %d", (int)(w.waypoint_plan_index));
				ROS_INFO("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- plan_valid: %d", (int)(w.plan_valid));
				ROS_INFO("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- execution_valid: %d", (int)(w.execution_valid));

				trajectory_info_[status->trajectory_name][ee] = new affordance_template_msgs::WaypointInfo();
				trajectory_info_[status->trajectory_name][ee]->id = w.id;
				trajectory_info_[status->trajectory_name][ee]->end_effector_name = w.end_effector_name;
				trajectory_info_[status->trajectory_name][ee]->num_waypoints = w.num_waypoints;
				trajectory_info_[status->trajectory_name][ee]->waypoint_index = w.waypoint_index;
				trajectory_info_[status->trajectory_name][ee]->waypoint_plan_index = w.waypoint_plan_index;
				trajectory_info_[status->trajectory_name][ee]->plan_valid = w.plan_valid;
				trajectory_info_[status->trajectory_name][ee]->execution_valid = w.execution_valid;
			}

			return true;
		}

    protected:

    	std::string name_;
		int id_;
		TrajectoryWaypointInfo trajectory_info_;
		std::string current_trajectory_;

    };
}

#endif // AFFORDANCE_TEMPLATE_STATUS_INFO_HPP