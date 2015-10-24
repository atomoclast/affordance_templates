#ifndef _AFFORDANCE_TEMPLATE_H_
#define _AFFORDANCE_TEMPLATE_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <utils/marker_helper.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>

#include <affordance_template_markers/robot_interface.h>

#include <affordance_template_library/affordance_template_structure.h>
#include <affordance_template_library/affordance_template_parser.h>


namespace affordance_template 
{

  struct WaypointTrajectoryFlags {
    bool run_backwards;
    bool auto_execute;
    bool loop;
    std::map<std::string, bool> controls_on;
  };

  struct PlanStatus {
    std::vector<int> sequence_ids;
    std::vector<geometry_msgs::PoseStamped> sequence_poses;
    bool backwards;
    bool plan_valid;
    bool exec_valid;
    bool direct;
    int current_idx = -1;
    int goal_idx = -1;
  };


  class AffordanceTemplate
  { 

  public:
    
    AffordanceTemplate(){} // default constructor
    AffordanceTemplate(const ros::NodeHandle nh, 
                       boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server, 
                       std::string robot_name, 
                       std::string template_type,
                       int id);
    AffordanceTemplate(const ros::NodeHandle nh, 
                       boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server, 
                       boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface,
                       std::string robot_name, 
                       std::string template_type,
                       int id);
    ~AffordanceTemplate();

    void run();

    void setRobotInterface(boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface);
    

    bool loadFromFile(std::string filename, geometry_msgs::Pose pose, affordance_template_object::AffordanceTemplateStructure &structure);

    inline int getID() { return id_; }
    inline std::string getType() { return template_type_; }
    inline std::string getCurrentTrajectory() { return current_trajectory_; }
    inline affordance_template_object::AffordanceTemplateStructure getCurrentStructure() { return initial_structure_; }
    inline boost::shared_ptr<affordance_template_markers::RobotInterface> getRobotInterface() { return robot_interface_; }

    int getNumWaypoints(const affordance_template_object::AffordanceTemplateStructure structure, const std::string traj_name, const int ee_id);

    std::map<std::string, bool> planPathToWaypoints(const std::vector<std::string>&, int, bool, bool); // list of ee names, steps, direct, backwards; return map of bools keyed on EE name
    bool moveToWaypoints(const std::vector<std::string>&); // list of ee waypoints to move to, return true if all waypoints were valid
    
    // TODO -- called from server/interface.cpp
    bool trajectoryHasEE(const std::string&, const std::string&); // trajectory name, ee name
    bool validWaypointPlan(const std::vector<std::string>&, const std::string&); //vector of ee names, trajectory name
    bool saveToDisk(const std::string&, const std::string&, const std::string&, bool); // filename, image, new key/class name, save_scale_updates bool
    bool addTrajectory(const std::string&); // trajectory name
    bool setTrajectory(const std::string&); // trajectory name, from the service msg it looks like if it's blank then set to current??
    bool scaleObject(const std::string&, double, double); // object name, scale factor, ee scale factor
    
  private:

    // menu config will hold the menu text as well as bool for whether it should have a checkbox
    typedef std::pair<std::string, bool> MenuConfig;

    // this will handle menus. first item is group name, vector list is nested menu text(s)
    typedef std::map<std::string, std::vector<std::string> > MenuHandleKey;

    // this is a storage DS to store "named" pose info. i.e., frame 'first' in pose.header.frame_id frame 
    typedef std::pair<std::string, geometry_msgs::PoseStamped> FrameInfo;

    typedef std::map<std::string, PlanStatus> EndEffectorPlanStatusMap;
    typedef std::map<std::string, EndEffectorPlanStatusMap> TrajectoryPlanStatus;

    ros::NodeHandle nh_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    // bookkeeping and IDs
    std::string robot_name_;
    std::string template_type_;
    std::string name_;
    std::string key_;
    std::string root_object_;
    std::string root_frame_;
    int id_;
    double loop_rate_;
    bool object_controls_display_on_;

    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface_;

    std::map<std::string, visualization_msgs::InteractiveMarker> int_markers_;
    std::map<std::string, interactive_markers::MenuHandler> marker_menus_;
    std::map<MenuHandleKey, interactive_markers::MenuHandler::EntryHandle> group_menu_handles_;
    std::map<std::string, AffordanceTemplate::FrameInfo> frame_store_;

    std::vector<MenuConfig> object_menu_options_;
    std::vector<MenuConfig> waypoint_menu_options_;
      
    affordance_template_object::AffordanceTemplateParser at_parser_;
    affordance_template_object::AffordanceTemplateStructure initial_structure_;
    affordance_template_object::AffordanceTemplateStructure structure_;

    std::string current_trajectory_;

    std::map<std::string, double> object_scale_factor_;
    std::map<std::string, double> ee_scale_factor_;

    std::map<std::string, WaypointTrajectoryFlags> waypoint_flags_;
    TrajectoryPlanStatus plan_status_;

    std::string getRootObject() { return root_object_; }
    void setRootObject(std::string root_object) { root_object_ = root_object; }

    bool isValidTrajectory(affordance_template_object::Trajectory traj);
    bool setCurrentTrajectory(affordance_template_object::TrajectoryList traj_list, std::string traj); 
    bool getTrajectory(affordance_template_object::TrajectoryList traj_list, std::string traj_name, affordance_template_object::Trajectory &traj);

    void clearTrajectoryFlags();
    void setTrajectoryFlags(affordance_template_object::Trajectory traj);

    std::string appendID(std::string s);
    std::string createWaypointID(int ee_id, int wp_id);
    bool appendIDToStructure(affordance_template_object::AffordanceTemplateStructure &structure);
    int getEEIDfromWaypointName(const std::string wp_name);

    bool createFromStructure(affordance_template_object::AffordanceTemplateStructure structure, bool keep_poses=false, std::string traj="");
    bool createDisplayObjectsFromStructure(affordance_template_object::AffordanceTemplateStructure structure, bool keep_poses);
    bool createWaypointsFromStructure(affordance_template_object::AffordanceTemplateStructure structure, bool keep_poses);

    void addInteractiveMarker(visualization_msgs::InteractiveMarker m);
    void removeInteractiveMarker(std::string marker_name);
    void removeAllMarkers();

    void setupMenuOptions();
    void setupObjectMenu(affordance_template_object::AffordanceTemplateStructure structure, affordance_template_object::DisplayObject obj);
    void setupWaypointMenu(affordance_template_object::AffordanceTemplateStructure structure,  std::string name);
    void setupSimpleMenuItem(affordance_template_object::AffordanceTemplateStructure structure, const std::string& name, const std::string& menu_text, bool has_check_box);
    void setupTrajectoryMenu(affordance_template_object::AffordanceTemplateStructure structure, const std::string& name);
    void setupEndEffectorPoseMenu(const std::string& name);

    bool hasObjectFrame(std::string obj);
    bool hasWaypointFrame(std::string wp);

    bool isObject(std::string obj);
    bool isWaypoint(std::string wp);

    geometry_msgs::Pose originToPose(affordance_template_object::Origin origin);


    bool computePathSequence(affordance_template_object::AffordanceTemplateStructure structure, std::string traj_name, int ee_id, int idx, int steps, bool backwards, std::vector<int> &sequence_ids, int &next_path_idx);

    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);


  };
}

#endif