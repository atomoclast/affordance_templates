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

		bool endEffectorInTrajectory(std::string traj_name, std::string ee) {

			auto traj_search = trajectory_info_.find(traj_name);
    		if(traj_search == trajectory_info_.end()) {
    			return false;
    		}

			auto ee_search = trajectory_info_[traj_name].find(ee);
    		if(ee_search == trajectory_info_[traj_name].end()) {
    			return false;
    		}

    		return true;
		}

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

				ROS_DEBUG("AffordanceTemplateStatusInfo::updateTrajectoryStatus() -- adding waypoint info for ee: %s", ee.c_str());
				ROS_DEBUG("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- id: %d", w.id);
				ROS_DEBUG("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- num_waypoints: %d", (int)(w.num_waypoints));
				ROS_DEBUG("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- waypoint_index: %d", (int)(w.waypoint_index));
				ROS_DEBUG("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- waypoint_plan_index: %d", (int)(w.waypoint_plan_index));
				ROS_DEBUG("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- plan_valid: %d", (int)(w.plan_valid));
				ROS_DEBUG("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- execution_valid: %d", (int)(w.execution_valid));
				for(size_t idx=0; idx<w.compact_view.size(); idx++) {
					ROS_DEBUG("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- compact_view[%d]: %d", (int)idx, (int)(w.compact_view[(int)idx]));
				}

				trajectory_info_[status->trajectory_name][ee] = new affordance_template_msgs::WaypointInfo();
				trajectory_info_[status->trajectory_name][ee]->id = w.id;
				trajectory_info_[status->trajectory_name][ee]->end_effector_name = w.end_effector_name;
				trajectory_info_[status->trajectory_name][ee]->num_waypoints = w.num_waypoints;
				trajectory_info_[status->trajectory_name][ee]->waypoint_index = w.waypoint_index;
				trajectory_info_[status->trajectory_name][ee]->waypoint_plan_index = w.waypoint_plan_index;
				trajectory_info_[status->trajectory_name][ee]->plan_valid = w.plan_valid;
				trajectory_info_[status->trajectory_name][ee]->execution_valid = w.execution_valid;
				trajectory_info_[status->trajectory_name][ee]->compact_view = w.compact_view;
				for(size_t idx=0; idx<w.compact_view.size(); idx++) {
					ROS_DEBUG("AffordanceTemplateStatusInfo::updateTrajectoryStatus() ---- compact_view[%d]: %d", (int)idx, (int)(trajectory_info_[status->trajectory_name][ee]->compact_view[(int)idx]));
				}
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