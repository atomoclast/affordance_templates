#include <affordance_template_markers/robot_interface.h>

using namespace affordance_template_markers;

RobotInterface::RobotInterface(const std::string &_joint_states_topic)
{
  reset();
  ros::NodeHandle nh;
  nh.subscribe(_joint_states_topic, 10, &RobotInterface::jointStateCallback, this);
}

RobotInterface::~RobotInterface() 
{
  delete robot_planner_;
}

bool RobotInterface::load(const std::string &yaml)
{
  reset();

  ROS_INFO("[RobotInterface::load] loading with input yaml %s", yaml.c_str());
  
  try
  {
    std::vector<std::string> tokens;
    boost::split(tokens, yaml, boost::is_any_of("/"));
    std::string yaml_base = tokens.back();
    ROS_WARN_STREAM("yaml base is "<<yaml_base);

    ros::NodeHandle nh;
    nh.setParam("/affordance_templates/robot_yaml", yaml_base);
   
    YAML::Node yaml_doc = YAML::LoadFile(yaml);

    robot_config_.filename = yaml;
    robot_config_.name = yaml_doc["robot_name"].as<std::string>();
    robot_config_.config_package = yaml_doc["config_package"].as<std::string>();
    robot_config_.config_file = yaml_doc["config_file"].as<std::string>();
    robot_config_.planner_type = yaml_doc["planner_type"].as<std::string>();
    robot_config_.frame_id = yaml_doc["frame_id"].as<std::string>();

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

    // TODO
    // more python to convert to C++ once we get EEConfig() class up
    // for ee in self.yaml_config['end_effector_group_map']:
    //     ee_config = EndEffectorConfig()
    //     ee_config.name = ee['name']
    //     ee_config.id = ee['id']
    //     ee_config.pose_offset = Pose()

    //     try :
    //         q = (kdl.Rotation.RPY(ee['pose_offset'][3],ee['pose_offset'][4],ee['pose_offset'][5])).GetQuaternion()
    //         ee_config.pose_offset.position.x = float(ee['pose_offset'][0])
    //         ee_config.pose_offset.position.y = float(ee['pose_offset'][1])
    //         ee_config.pose_offset.position.z = float(ee['pose_offset'][2])
    //         ee_config.pose_offset.orientation.x = q[0]
    //         ee_config.pose_offset.orientation.y = q[1]
    //         ee_config.pose_offset.orientation.z = q[2]
    //         ee_config.pose_offset.orientation.w = q[3]
    //     except :
    //         ee_config.pose_offset.orientation.w = 1.0
    //     self.manipulator_pose_map[ee['name']] = ee_config.pose_offset
        
    //     ee_config.tool_offset = Pose()
    //     try :
    //         q = (kdl.Rotation.RPY(ee['tool_offset'][3],ee['tool_offset'][4],ee['tool_offset'][5])).GetQuaternion()
    //         ee_config.tool_offset.position.x = float(ee['tool_offset'][0])
    //         ee_config.tool_offset.position.y = float(ee['tool_offset'][1])
    //         ee_config.tool_offset.position.z = float(ee['tool_offset'][2])
    //         ee_config.tool_offset.orientation.x = q[0]
    //         ee_config.tool_offset.orientation.y = q[1]
    //         ee_config.tool_offset.orientation.z = q[2]
    //         ee_config.tool_offset.orientation.w = q[3]
    //     except :
    //         ee_config.tool_offset.orientation.w = 1.0
    //     self.tool_offset_map[ee['name']] = ee_config.tool_offset

    //     self.end_effector_names.append(ee['name'])
    //     self.end_effector_name_map[ee['id']] = ee['name']
    //     self.manipulator_id_map[ee['name']] = ee['id']
        
    //     self.robot_config.end_effectors.append(ee_config)

        // # if there is a tool offset
        // # t = geometry_msgs.msg.Pose()
        // # try :
        // #     q = (kdl.Rotation.RPY(ee['tool_offset'][3],ee['tool_offset'][4],ee['tool_offset'][5])).GetQuaternion()
        // #     t.position.x = float(ee['tool_offset'][0])
        // #     t.position.y = float(ee['tool_offset'][1])
        // #     t.position.z = float(ee['tool_offset'][2])
        // #     t.orientation.x = q[0]
        // #     t.orientation.y = q[1]
        // #     t.orientation.z = q[2]
        // #     t.orientation.w = q[3]
        // # except :
        // #     t.orientation.w = 1.0
        // # self.tool_offset_map[ee['name']] = t

    // for ee in self.yaml_config['end_effector_pose_map']:

    //     ee_pose_config = EndEffectorPoseData()

    //     ee_pose_config.group = ee['group']
    //     ee_pose_config.name = ee['name']
    //     ee_pose_config.id = ee['id']
        
    //     self.robot_config.end_effector_pose_data.append(ee_pose_config)

    //     if not ee['group'] in self.end_effector_pose_map:
    //         self.end_effector_pose_map[ee['group']] = {}
    //         self.end_effector_id_map[ee['group']] = {}
    //     self.end_effector_pose_map[ee['group']][ee['name']] = int(ee['id'])
    //     self.end_effector_id_map[ee['group']][int(ee['id'])] = ee['name']
    
  }
  catch(...)
  {
    ROS_FATAL("[RobotInterface::load] error opening config file!!");
    return false;
  }

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
    return false;
    // robot_planner_ = AtlasPathPlanner(robot_config_.name, config_file_);
  }
  else if (robot_config_.planner_type == "hybrid")
  {
    return false;
    // robot_planner_ = AtlasHybridPathPlanner(robot_config_.name, config_file_);
  }
  else
  {
    ROS_FATAL("[RobotInterface::configure] %s is an unrecognized planner type!!", robot_config_.planner_type.c_str());
    return false;
  }

  // root_frame_ = robot_planner_->getRobotPlanningFrame(); TODO - doesn't exist in planner_interface
  ee_groups_ = robot_planner_->getEndEffectorNames();

  for (auto g : ee_groups_)
  {
    if (std::find(ee_groups_.begin(), ee_groups_.end(), g) == ee_groups_.end())
    {
      ROS_FATAL("[RobotInterface::configure] group %s is  not in end effector groups!!", g.c_str());
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

        // try{
        //   listener.lookupTransform("/turtle2", "/turtle1",  
        //                            ros::Time(0), transform);
        // }
        // catch (tf::TransformException ex){
        //   ROS_ERROR("%s",ex.what());
        //   ros::Duration(1.0).sleep();
        // }

        // @seth -- still todo with python-->cpp
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
      ROS_INFO("[RobotInterface::configure] ading stored pose %s to group %s", name.c_str(), g.c_str());
      sensor_msgs::JointState state;
      robot_planner_->getStoredGroupState(g, name, state);
      stored_poses_[std::make_pair(g, name)] = state;
    }
  }

  // todo -- @seth figure out what the hell a gripper_action looks like
  // I am confused because this is a python list but other places (other classes) is referenced as a dictionary
  // this is the grossest gross c++ container ever
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
  return ""; //TODO
//   #include <ros/package.h>

// ...

//   std::string path = ros::package::getPath("roslib");
//   using package::V_string;
//   V_string packages;
//   ros::package::getAll(packages);
}