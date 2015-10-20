#ifndef _ROBOT_INTERFACE_H_
#define _ROBOT_INTERFACE_H_

#include <fstream>
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <affordance_template_msgs/RobotConfig.h>
#include <affordance_template_msgs/EndEffectorConfig.h>
#include <affordance_template_msgs/EndEffectorPoseData.h>
#include <planner_interface/planner_interface.h>
#include <moveit_planner/moveit_planner.h>

namespace affordance_template_markers 
{
  class RobotInterface
  {
    void jointStateCallback(const sensor_msgs::JointState&);

    sensor_msgs::JointState joint_data_;
    affordance_template_msgs::RobotConfig robot_config_;
    tf::TransformListener listener_;
    planner_interface::PlannerInterface *robot_planner_; 
    
    bool configured_;
    bool reload_attempted_;
    bool planner_created_;
    std::string config_file_;
    std::string root_frame_;
    
    std::vector<std::string> ee_names_;
    std::vector<std::string> ee_groups_;
    
    // std::map<std::string, robot_interaction_tools::EndEffectorHelper> ee_link_data_; //TODO
    std::map<int, std::string> ee_name_map_;
    // std::map<std::string, > ee_markers_;
    std::map<std::string, geometry_msgs::Pose> tool_offset_map_;
    std::map<std::string, int> manipulator_id_map_;
    std::map<std::string, geometry_msgs::Pose> manipulator_pose_map_;

    std::map<std::pair<std::string, int>, std::string> ee_id_map_;
    std::map<std::pair<std::string, std::string>, int> ee_pose_map_;
    std::map<std::pair<std::string, std::string>, sensor_msgs::JointState> stored_poses_;


  public:
    RobotInterface();
    RobotInterface(const std::string &_joint_states_topic); 
    ~RobotInterface();

    bool load(const std::string&);
    bool load(const affordance_template_msgs::RobotConfig&);
    bool configure();

    inline affordance_template_msgs::RobotConfig getRobotConfig() { return robot_config_; }
    inline std::map<int, std::string> getEENameMap() { return ee_name_map_; }
    
    std::string getEEName(const int);
    int getEEId(const std::string&);
    std::string getManipulator(const std::string&); // is this right??
    std::string getPkgPath(const std::string&);
    
    void tearDown();
    void reset();

  };
}

#endif