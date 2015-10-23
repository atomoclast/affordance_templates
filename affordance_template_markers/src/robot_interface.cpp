#include <affordance_template_markers/robot_interface.h>

using namespace affordance_template_markers;
using namespace end_effector_helper;

RobotInterface::RobotInterface()
{
  reset();
  ros::NodeHandle nh;
  nh.subscribe("joint_states", 10, &RobotInterface::jointStateCallback, this);
  reload_attempted_ = false; 
}

RobotInterface::RobotInterface(const std::string &_joint_states_topic="joint_states")
{
  reset();
  ros::NodeHandle nh;
  nh.subscribe(_joint_states_topic, 10, &RobotInterface::jointStateCallback, this);
  reload_attempted_ = false;
}

RobotInterface::~RobotInterface() 
{
  if (planner_created_)
    delete robot_planner_;
}

bool RobotInterface::load(const std::string &yaml)
{
  ROS_INFO("[RobotInterface::load] loading with input yaml: %s", yaml.c_str());

  reset();
 
  try
  {
    std::vector<std::string> tokens;
    boost::split(tokens, yaml, boost::is_any_of("/"));
    std::string yaml_base = tokens.back();

    ros::NodeHandle nh;
    nh.setParam("/affordance_templates/robot_yaml", yaml);
   
    YAML::Node yaml_doc = YAML::LoadFile(yaml);

    robot_config_.filename = yaml;
    robot_config_.name = yaml_doc["robot_name"].as<std::string>();
    ROS_INFO_STREAM("[RobotInterface::load] parsed name: "<< robot_config_.name);
    robot_config_.config_package = yaml_doc["config_package"].as<std::string>();
    ROS_INFO_STREAM("[RobotInterface::load] parsed config_package: "<< robot_config_.config_package);
    robot_config_.config_file = yaml_doc["config_file"].as<std::string>();
    ROS_INFO_STREAM("[RobotInterface::load] parsed config_file: "<< robot_config_.config_file);
    robot_config_.planner_type = yaml_doc["planner_type"].as<std::string>();
    ROS_INFO_STREAM("[RobotInterface::load] parsed planner_type: "<< robot_config_.planner_type);
    robot_config_.frame_id = yaml_doc["frame_id"].as<std::string>();
    ROS_INFO_STREAM("[RobotInterface::load] parsed frame_id: "<< robot_config_.frame_id);

    tf::Quaternion q;
    q.setRPY(yaml_doc["root_offset"][3].as<double>(), yaml_doc["root_offset"][4].as<double>(), yaml_doc["root_offset"][5].as<double>());
    geometry_msgs::Pose p;
    p.position.x = yaml_doc["root_offset"][0].as<double>();
    p.position.y = yaml_doc["root_offset"][1].as<double>();
    p.position.z = yaml_doc["root_offset"][2].as<double>();
    p.orientation.x = q[0];
    p.orientation.y = q[1];
    p.orientation.z = q[2];
    p.orientation.w = q[3];
    robot_config_.root_offset = p;
    ROS_INFO_STREAM("[RobotInterface::load] parsed root_offset position: "<<p.position.x<<" "<<p.position.y<<" "<<p.position.z);
    ROS_INFO_STREAM("[RobotInterface::load] parsed root_offset orientation: "<<p.orientation.x<<" "<<p.orientation.y<<" "<<p.orientation.z<<" "<<p.orientation.w);

    YAML::Node ee_group = yaml_doc["end_effector_group_map"];
    ROS_INFO_STREAM("[RobotInterface::load] parsing end effector group map");
    for (auto ee : ee_group)
    {
      affordance_template_msgs::EndEffectorConfig ee_config;
      ee_config.name = ee["name"].as<std::string>();
      ROS_INFO_STREAM("[RobotInterface::load]     parsed end effector name: "<< ee_config.name.c_str());
      ee_config.id = ee["id"].as<int>();
      ROS_INFO_STREAM("[RobotInterface::load] \t\tparsed end effector id: "<< int(ee_config.id));

      tf::Quaternion q;
      geometry_msgs::Pose po;
      try
      {
        YAML::Node pose = ee["pose_offset"];
        q.setRPY(pose[3].as<double>(), pose[4].as<double>(), pose[5].as<double>());
        po.position.x = pose[0].as<double>();
        po.position.y = pose[1].as<double>();
        po.position.z = pose[2].as<double>();
        po.orientation.x = q[0];
        po.orientation.y = q[1];
        po.orientation.z = q[2];
        po.orientation.w = q[3];
        ee_config.pose_offset = po;
        ROS_INFO_STREAM("[RobotInterface::load] \t\tparsed pose_offset position: "<<po.position.x<<" "<<po.position.y<<" "<<po.position.z);
        ROS_INFO_STREAM("[RobotInterface::load] \t\tparsed pose_offset orientation: "<<po.orientation.x<<" "<<po.orientation.y<<" "<<po.orientation.z<<" "<<po.orientation.w);
      }
      catch(...)
      {
        po.orientation.w = 1.0;  
      }
      manipulator_pose_map_[ee_config.name] = ee_config.pose_offset;

      geometry_msgs::Pose to;
      try
      {
        YAML::Node tool = ee["tool_offset"];
        q.setRPY(tool[3].as<double>(), tool[4].as<double>(), tool[5].as<double>());
        to.position.x = tool[0].as<double>();
        to.position.y = tool[1].as<double>();
        to.position.z = tool[2].as<double>();
        to.orientation.x = q[0];
        to.orientation.y = q[1];
        to.orientation.z = q[2];
        to.orientation.w = q[3];
        ee_config.tool_offset = to;
        ROS_INFO_STREAM("[RobotInterface::load] \t\tparsed tool_offset position: "<<to.position.x<<" "<<to.position.y<<" "<<to.position.z);
        ROS_INFO_STREAM("[RobotInterface::load] \t\tparsed tool_offset orientation: "<<to.orientation.x<<" "<<to.orientation.y<<" "<<to.orientation.z<<" "<<to.orientation.w);
      }
      catch(...)
      {
        to.orientation.w = 1.0;  
      }
      //tool_offset_map_[ee_config.name] = ee_config.tool_offset;
      tool_offset_map_[ee_config.name] = to;

      ee_names_.push_back(ee_config.name);
      ee_name_map_[ee_config.id] = ee_config.name;
      manipulator_id_map_[ee_config.name] = ee_config.id;
      robot_config_.end_effectors.push_back(ee_config);
    } 

    YAML::Node ee_pose = yaml_doc["end_effector_pose_map"];
    ROS_INFO_STREAM("[RobotInterface::load] parsing end effector pose map");
    for (auto ee : ee_pose)
    {
      affordance_template_msgs::EndEffectorPoseData ee_pose;

      ee_pose.name = ee["name"].as<std::string>();
      ROS_INFO_STREAM("[RobotInterface::load]     parsed end effector name: "<< ee_pose.name.c_str());
      ee_pose.group = ee["group"].as<std::string>();
      ROS_INFO_STREAM("[RobotInterface::load] \t\tparsed end effector group: "<< ee_pose.group.c_str());
      ee_pose.id = ee["id"].as<int>();
      ROS_INFO_STREAM("[RobotInterface::load] \t\tparsed end effector id: "<< int(ee_pose.id));

      robot_config_.end_effector_pose_data.push_back(ee_pose);

      ee_pose_map_[std::make_pair(ee_pose.group, ee_pose.name)] = ee_pose.id;
      ee_id_map_[std::make_pair(ee_pose.group, ee_pose.id)] = ee_pose.name;
    }

    // TODO 
    // still need this gripper action thing
    // self.robot_config.gripper_action = []
    // try :
    //     ga = self.yaml_config['gripper_action']
    //     for g in ga :
    //         gm = GripperActionMap()
    //         gm.name = g['name']
    //         gm.action = g['action']
    //         self.robot_config.gripper_action.append(gm)
    //         rospy.logwarn(str("RobotInterface() -- found " + g['name'] + " gripper action : " + g['action']))
    // except :
    //     pass 
  }
  catch(...)
  {
    if (!reload_attempted_)
    {
      std::string path = ros::package::getPath("affordance_template_library");
      path += "/robots/" + yaml;
      ROS_INFO_STREAM("[RobotInterface::load] couldn't open file!! retrying with full path "<<path);

      reload_attempted_ = true;
      bool loaded = load(path);
      reload_attempted_ = false;
      return loaded;
    }
    else
    {
      ROS_WARN("[RobotInterface::load] did not successfully load from yaml file");
      reload_attempted_ = false;
      return false;
    }
  }

  configured_ = true;
  ROS_INFO("[RobotInterface::load] successfully loaded from yaml file");
  
  return true;
}

bool RobotInterface::load(const affordance_template_msgs::RobotConfig &config)
{
  reset();
  
  ROS_INFO("[RobotInterface::load] loading with robot_config msg");
  robot_config_ = config;

  ros::NodeHandle nh;
  nh.setParam("/affordance_templates/robot_yaml", "");

  for (auto config : robot_config_.end_effectors)
  {
    ee_names_.push_back(config.name);
    ee_name_map_[config.id] = config.name;
    manipulator_id_map_[config.name] = config.id;
    manipulator_pose_map_[config.name] = config.pose_offset;
    tool_offset_map_[config.name] = config.tool_offset;
  }

  for (auto pose : robot_config_.end_effector_pose_data)
  {
    ee_id_map_[std::make_pair(pose.group, pose.id)] = pose.name;
    ee_pose_map_[std::make_pair(pose.group, pose.name)] = pose.id;
  }

  configured_ = true;
  ROS_INFO("[RobotInterface::load] successfully loaded from robot_config msg");
  
  return true;
}

void RobotInterface::jointStateCallback(const sensor_msgs::JointState &state)
{
  joint_data_ = state;
}

bool RobotInterface::configure() // TODO
{
  ROS_INFO("[RobotInterface::configure] robot name: %s", robot_config_.name.c_str());
  ROS_INFO("[RobotInterface::configure] package: %s", config_file_.c_str());
  ROS_INFO("[RobotInterface::configure] using planner type: %s", robot_config_.planner_type.c_str());

  if (robot_config_.planner_type == "moveit")
  {
    // robot_planner_ = new moveit_planner::MoveItPlanner(robot_config_.name, config_file_); // TODO?? not implemented this way with cpp rewrite
    robot_planner_ = new moveit_planner::MoveItPlanner();
  }
  else if (robot_config_.planner_type == "atlas")
  {
    ROS_WARN("[RobotInterface::configure] nothing for atlas planner yet...");
    return false;
    // robot_planner_ = AtlasPathPlanner(robot_config_.name, config_file_);
  }
  else if (robot_config_.planner_type == "hybrid")
  {
    ROS_WARN("[RobotInterface::configure] nothing for hybrid planner yet...");
    return false;
    // robot_planner_ = AtlasHybridPathPlanner(robot_config_.name, config_file_);
  }
  else
  {
    ROS_FATAL("[RobotInterface::configure] %s is an unrecognized planner type!!", robot_config_.planner_type.c_str());
    return false;
  }

  ros::NodeHandle nh;
  robot_planner_->initialize(nh, robot_config_.name);

  // root_frame_ = robot_planner_->getRobotPlanningFrame(); TODO - doesn't exist in planner_interface
  ee_groups_ = robot_planner_->getEndEffectorNames();

  // get actual planning SRDF groups for EEs
  std::vector<std::string> ee_planning_groups;
  for (auto g : ee_groups_)
  {
    std::string pg;
    robot_planner_->getRDFModel()->getEndEffectorPlanningGroup(g, pg);
    ee_planning_groups.push_back(pg);
  }
  for (auto g : ee_planning_groups)
  {
    ROS_INFO("[RobotInterface::configure] setting up ee group: %s", g.c_str());

    if (std::find(ee_planning_groups.begin(), ee_planning_groups.end(), g) == ee_planning_groups.end())
    {
      ROS_FATAL("[RobotInterface::configure] group %s is not in end effector groups!!", g.c_str());
      return false;
    }

    try
    {
      std::vector<double> pos_tol, orientation_tol; // TODO TEMP, need overloaded method  for just group name and group type?? in planner interface???
      pos_tol.push_back(0.1);
      orientation_tol.push_back(0.1);
      if (robot_planner_->addPlanningGroup(g, "endeffector", 0.05, pos_tol, orientation_tol)) // was addPlanningGroup(g, "endeffector")
      {
        // robot_planner_->setGoalJointTolerance(g, 0.05);
        std::string ee_root_frame = robot_planner_->getControlFrame(g);
        ROS_INFO("[RobotInterface::configure] control frame: %s", ee_root_frame.c_str());

        // @seth -- still todo with python-->cpp

        EndEffectorHelperConstPtr ee(new EndEffectorHelper(g, ee_root_frame, robot_planner_->getRDFModel()));
        ee_link_data_[g] = ee;

        // self.end_effector_link_data[g] = EndEffectorHelper(self.robot_config.name, g, ee_root_frame, self.tf_listener)
        // self.end_effector_link_data[g].populate_data(self.robot_planner.get_group_links(g), self.robot_planner.get_urdf_model(), self.robot_planner.get_srdf_model())
        // rospy.sleep(1)
        // self.end_effector_markers[g] = self.end_effector_link_data[g].get_current_position_marker_array(scale=1.0,color=(1,1,1,0.5))
        // pg = self.robot_planner.get_srdf_model().get_end_effector_parent_group(g)
        // if not pg == None :
        //     rospy.loginfo(str("RobotInterface::configure() -- trying to add group: " + pg))
        //     self.robot_planner.add_planning_group(pg, group_type="cartesian")
        //     self.robot_planner.set_display_mode(pg, "all_points")
        //     self.robot_planner.set_goal_position_tolerances(pg, [.01]*3)
        //     self.robot_planner.set_goal_orientation_tolerances(pg, [.03]*3)
        //     self.robot_planner.set_goal_joint_tolerance(pg, 0.05)

        //     self.stored_poses[pg] = {}
        //     for state_name in self.robot_planner.get_stored_state_list(pg) :
        //         rospy.loginfo(str("RobotInterface::configure() adding stored pose \'" + state_name + "\' to group \'" + pg + "\'"))
        //         self.stored_poses[pg][state_name] = self.robot_planner.get_stored_group_state(pg, state_name)

        // else :
            // rospy.logerror(str("RobotInterface::configure -- no manipulator group found for end-effector: " + g))

      }
      else
      {
        ROS_ERROR("[RobotInterface::configure] problem adding end effector group %s to planner", g.c_str());
      }
    } 
    catch(...)
    {
      ROS_WARN("[RobotInterface::configure] skipping group %s", g.c_str());
    }

    std::vector<std::string> state_list;
    robot_planner_->getStoredStateList(g, state_list);
    for (auto name : state_list)
    {
      ROS_INFO("[RobotInterface::configure] adding stored pose %s to group %s", name.c_str(), g.c_str());
      sensor_msgs::JointState state;
      robot_planner_->getStoredGroupState(g, name, state);
      stored_poses_[std::make_pair(g, name)] = state;
    }
  }

  // todo -- @seth figure out what the hell a gripper_action looks like
  // I am confused because this is a python list but other places (other classes) is referenced as a dictionary
  // this is the grossest gross c++ container ever; I hate you Steve
  // if (robot_config_.gripper_action)
  // {
  //   std::vector<std::map<std::pair<std::string, std::string>, std::pair<std::string, int> action;
  //   for (auto g : robot_config_.gripper_action)
  //   {
  //     action.push_back();
  //   }
  //   robot_planner_->set_gripper_actions(action);
  // }

  robot_planner_->printBasicInfo();
  ROS_INFO("[RobotInterface::configure] successfully configured RobotInterface");
  return true;
}

void RobotInterface::reset()
{
  planner_created_ = false;
  configured_ = false;
  ee_names_.clear();
  ee_groups_.clear();
  ee_id_map_.clear();
  // ee_markers_.clear(); // TODO
  ee_name_map_.clear();
  ee_pose_map_.clear();
  // ee_link_data_.clear(); // TODO
  // stored_poses_.clear(); // TODO
  tool_offset_map_.clear();
  manipulator_id_map_.clear();
  manipulator_pose_map_.clear();
}

void RobotInterface::tearDown()
{
  // for (auto ee : ee_link_data_) // TODO
  //   ee.stop_offset_update_thread();
  robot_planner_->tearDown();
  configured_ = false;
}

// #########
//  getters
// #########

std::string RobotInterface::getEEName(const int id)
{
  return ee_name_map_[id];
}

int RobotInterface::getEEId(const std::string &name)
{
  return manipulator_id_map_[name];
}

std::string RobotInterface::getManipulator(const std::string &ee)
{
  // return robot_planner_->get_srdf_model().get_end_effector_parent_group(ee);
  return "";//TODO
} 

std::string RobotInterface::getPkgPath(const std::string &pkg)
{
  return ros::package::getPath(pkg);
}

// tester main
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tester_for_robot_interface_class");

  RobotInterface ri("test_joint_states");
  ri.load("r2_upperbody.yaml");

  return 0;
}

std::map<std::string, int> RobotInterface::getEEPoseIDMap(std::string name) 
{
  std::map<std::string, int> pose_id_map;
  for(auto key : ee_pose_map_) {
    if(key.first.first == name) {
      std::string pose_name = key.first.second;
      pose_id_map[pose_name] = ee_pose_map_[key.first];
    }
  }
  return pose_id_map;
}

std::map<int, std::string> RobotInterface::getEEPoseNameMap(std::string name) 
{
  std::map<int, std::string> pose_name_map;
  for(auto key : ee_id_map_) {
    if(key.first.first == name) {
      int pose_id = key.first.second;
      pose_name_map[pose_id] = ee_id_map_[key.first];
    }
  }
  return pose_name_map;
}

bool RobotInterface::getEELinkData(std::string group_name, end_effector_helper::EndEffectorHelperConstPtr &link_data) 
{
  // for(auto g: ee_link_data_) {
  //   std::cout << "has link data for " << g.first << std::endl;
  // }
  if(ee_link_data_.find(group_name) != std::end(ee_link_data_)) {
    link_data = ee_link_data_[group_name];
  } else{
    return false;
  }
  return true;
}

std::vector<std::string> RobotInterface::getEEPoseNames(std::string name) 
{
  std::vector<std::string> pose_names;
  for(auto &k : ee_id_map_) {
    if(k.first.first == name) {
      std::string p = k.second;
      if(std::find(pose_names.begin(), pose_names.end(), p) == pose_names.end()) {
        pose_names.push_back(p);
      }
    }
  }
  return pose_names;
}

