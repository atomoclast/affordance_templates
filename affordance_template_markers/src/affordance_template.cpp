#include <affordance_template_markers/affordance_template.h>

using namespace affordance_template;
using namespace affordance_template_object;
using namespace affordance_template_markers;


AffordanceTemplate::AffordanceTemplate(const ros::NodeHandle nh, 
                                        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,  
                                        std::string robot_name, 
                                        std::string template_type,
                                        int id) :
  nh_(nh),
  server_(server),
  robot_name_(robot_name),
  template_type_(template_type),
  id_(id),
  root_object_(""),
  loop_rate_(50.0),
  object_controls_display_on_(true)
{
  ROS_INFO("AffordanceTemplate::init() -- Done Creating new AffordanceTemplate of type %s for robot: %s", template_type_.c_str(), robot_name_.c_str());
  name_ = template_type_ + ":" + std::to_string(id);

  ros::AsyncSpinner spinner(1.0/loop_rate_);
  spinner.start();

  setupMenuOptions();

  // set to false when template gets destroyed, otherwise we can get a dangling pointer
  running_ = true;
  
  boost::thread spin_thread_(boost::bind(&AffordanceTemplate::run, this));
}


AffordanceTemplate::AffordanceTemplate(const ros::NodeHandle nh, 
                                        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,  
                                        boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface,
                                        std::string robot_name, 
                                        std::string template_type,
                                        int id) :
  AffordanceTemplate(nh, server, robot_name, template_type, id)
{
  setRobotInterface(robot_interface);
}


AffordanceTemplate::~AffordanceTemplate() 
{
	
}

void AffordanceTemplate::setRobotInterface(boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface)
{
  robot_interface_ = robot_interface;
}


void AffordanceTemplate::setupMenuOptions() 
{

  waypoint_menu_options_.clear();
  waypoint_menu_options_.push_back(MenuConfig("Change End-Effector Pose", false));
  waypoint_menu_options_.push_back(MenuConfig("Hide Controls", true));
  waypoint_menu_options_.push_back(MenuConfig("Add Waypoint Before", false));
  waypoint_menu_options_.push_back(MenuConfig("Add Waypoint After", false));
  waypoint_menu_options_.push_back(MenuConfig("Delete Waypoint", false));
  waypoint_menu_options_.push_back(MenuConfig("Move Forward", false));
  waypoint_menu_options_.push_back(MenuConfig("Move Back", false));
  
  object_menu_options_.clear();
  object_menu_options_.push_back(MenuConfig("Add Waypoint Before", false));
  object_menu_options_.push_back(MenuConfig("Add Waypoint After", false));
  object_menu_options_.push_back(MenuConfig("Reset", false));
  object_menu_options_.push_back(MenuConfig("Save", false));
  object_menu_options_.push_back(MenuConfig("Hide Controls", true));
  object_menu_options_.push_back(MenuConfig("Choose Trajectory", false));
  object_menu_options_.push_back(MenuConfig("Plan Test", false));
  object_menu_options_.push_back(MenuConfig("Execute Test", false));

}

bool AffordanceTemplate::loadFromFile(std::string filename, geometry_msgs::Pose pose, AffordanceTemplateStructure &structure)
{
  at_parser_.loadFromFile(filename, structure);

  // store copies in class, one as backup to reset/restore later
  initial_structure_ = structure;
  structure_ = structure;

  bool append = appendIDToStructure(structure_);
  bool created = createFromStructure(structure_);

  return (append && created);
}

bool AffordanceTemplate::saveToDisk(std::string& filename, const std::string& image, const std::string& key, bool save_scale_updates)
{
  std::string class_type = template_type_;
  if (filename.empty())
    filename = template_type_ + ".json";
  else
  {
    std::vector<std::string> keys;
    boost::split(keys, filename, boost::is_any_of("."));
    if (keys.size())
      class_type = keys.front();
  }

  std::string root = ros::package::getPath("affordance_template_library");
  if (root.empty())
      return false;
  root += "/templates";
  std::string output_path = root + "/" + filename;

  // if filename given is actually directory, return
  if (boost::filesystem::is_directory(output_path))
  {
    ROS_ERROR("[AffordanceTemplate::saveToDisk] error formatting filename!!");
    return false;
  }

  ROS_INFO("[AffordanceTemplate::saveToDisk] writing template to file: %s", output_path.c_str());

  if (!boost::filesystem::exists(output_path))
    ROS_WARN("[AffordanceTemplate::saveToDisk] no file found with name: %s. cannot create backup.", filename.c_str());
  else
  {
    // count how many bak files we have for this particular template type
    boost::filesystem::recursive_directory_iterator dir_it(root);
    boost::filesystem::recursive_directory_iterator end_it;
    int bak_counter = 0;
    while (dir_it != end_it)
    {

      if (std::string(dir_it->path().string()).find(".bak") != std::string::npos
          && std::string(dir_it->path().string()).find(class_type) != std::string::npos)
        ++bak_counter;
      ++dir_it;
    }

    // copy current class_type.json into .bak
    std::string bak_path = "";
    if (bak_counter > 0)
    {
      ROS_INFO("[AffordanceTemplate::saveToDisk] creating backup file: %s.bak%d", filename.c_str(), bak_counter);
      bak_path = output_path + ".bak" + std::to_string(bak_counter);
    }
    else
    {
      ROS_INFO("[AffordanceTemplate::saveToDisk] creating backup file: %s.bak", filename.c_str());
      bak_path = output_path + ".bak";
    }
    boost::filesystem::copy_file(output_path, bak_path, boost::filesystem::copy_option::overwrite_if_exists);
  }

  // set structure key as the new key name
  std::vector<std::string> keys;
  boost::split(keys, key, boost::is_any_of(":"));
  if (keys.size())
    structure_.name = keys.front();

  return at_parser_.saveToFile(output_path, structure_);
}

bool AffordanceTemplate::appendIDToStructure(AffordanceTemplateStructure &structure) 
{
  structure.name = appendID(structure.name);
  for(auto &obj : structure.display_objects) {
      obj.name = appendID(obj.name);
      if(obj.parent != "") {
        obj.parent = appendID(obj.parent);
      }
  }
  //       for traj in structure['end_effector_trajectory'] :
  //           for ee_group in traj['end_effector_group'] :
  //               for wp in ee_group['end_effector_waypoint'] :
  //                   wp['display_object'] = self.append_id(wp['display_object'])
  return true;
}
 
std::string AffordanceTemplate::createWaypointID(int ee_id, int wp_id)
{
  return std::to_string(ee_id) + "." + std::to_string(wp_id) + ":" + name_;
}

std::string AffordanceTemplate::appendID(std::string s) 
{
  return s + ":" + std::to_string(id_);
} 

bool AffordanceTemplate::addTrajectory(const std::string& trajectory_name) 
{
  affordance_template_object::Trajectory traj;
  traj.name = trajectory_name;
  structure_.ee_trajectories.push_back(traj);
  setTrajectory(trajectory_name);
  setupTrajectoryMenu(structure_, trajectory_name);
  return createFromStructure( structure_, false, trajectory_name);
}

bool AffordanceTemplate::getTrajectory(TrajectoryList& traj_list, std::string traj_name, Trajectory& traj) 
{
  for (auto &t: traj_list) {
    if(t.name == traj_name) {
      traj = t;
      return true;
    }
  }
  ROS_ERROR("AffordanceTemplate::getTrajectory() -- could not find %s in list of trajectories", traj_name.c_str());
  return false;
}

bool AffordanceTemplate::getTrajectoryPlan(const std::string& trajectory, const std::string& ee, PlanStatus& plan)
{
  if (plan_status_.find(trajectory) != plan_status_.end())
    if (plan_status_[trajectory].find(ee) != plan_status_[trajectory].end())
      plan = plan_status_[trajectory][ee];
    else
      return false;
  else
    return false;

  return true;
}

bool AffordanceTemplate::setTrajectory(const std::string& trajectory_name)
{
  return setCurrentTrajectory( getCurrentStructure().ee_trajectories, trajectory_name);
}

void AffordanceTemplate::clearTrajectoryFlags()
{
  waypoint_flags_.clear();
}

void AffordanceTemplate::setTrajectoryFlags(Trajectory traj) 
{  
  if(waypoint_flags_.find(traj.name) == std::end(waypoint_flags_)) {
    WaypointTrajectoryFlags wp_flags;

    wp_flags.run_backwards = false;
    wp_flags.loop = false;
    wp_flags.auto_execute = false;

    for(auto &ee : traj.ee_waypoint_list) {
      for(size_t idx=0; idx<ee.waypoints.size(); idx++) {
        std::string wp_name = createWaypointID(ee.id,idx);
        wp_flags.controls_on[wp_name] = false;
      }
    }
    waypoint_flags_[traj.name] = wp_flags;
  }
}

bool AffordanceTemplate::isValidTrajectory(Trajectory traj)  
{
  bool valid_trajectory = true;
  for(auto &g: traj.ee_waypoint_list) {
    for(auto &wp: g.waypoints) {
      if(robot_interface_->getEENameMap().find(g.id) == std::end(robot_interface_->getEENameMap())) {
        valid_trajectory = false;
        ROS_DEBUG("AffordanceTemplate::is_valid_trajectory() -- can't find ee ID: %d", g.id);
        return valid_trajectory;
      }
   }
 }
 return valid_trajectory;
} 

// set the default (current) traj to the input one.
// if no input request, find the first valid one
bool AffordanceTemplate::setCurrentTrajectory(TrajectoryList traj_list, std::string traj) 
{
  ROS_DEBUG("[AffordanceTemplate::setCurrentTrajectory] will attempt to set current trajectory to %s", traj.c_str());

  current_trajectory_ = "";
  if ( !traj.empty()) {
    for (auto &t: traj_list) {
      if(t.name == traj) {
        if(isValidTrajectory(t)) {
          current_trajectory_ = t.name;
          ROS_INFO("AffordanceTemplate::setCurrentTrajectory() -- setting current trajectory to: %s", current_trajectory_.c_str());
          break;
        }
      }
    }
  } 
  if ( current_trajectory_.empty()) {
    for (auto &t: traj_list) {
      if(isValidTrajectory(t)) {
        current_trajectory_ = t.name;
        if(traj!=current_trajectory_) {
          ROS_WARN("AffordanceTemplate::setCurrentTrajectory() -- \'%s\' not valid, setting current trajectory to: %s", traj.c_str(), current_trajectory_.c_str());
        } else {
          ROS_INFO("AffordanceTemplate::setCurrentTrajectory() -- setting current trajectory to: %s", current_trajectory_.c_str());          
        }
        break;
      }
    }
  } 
  
  if (current_trajectory_.empty()) // still no valid traj found
  {
    ROS_ERROR("AffordanceTemplate::createDisplayObjectsFromStructure() -- no valid trajectory found");
    return false;
  }
  return true;
}


bool AffordanceTemplate::createFromStructure(AffordanceTemplateStructure structure, bool keep_poses, std::string traj) 
{
  ROS_INFO("AffordanceTemplate::createFromStructure() -- %s", template_type_.c_str());
  if(setCurrentTrajectory(structure.ee_trajectories, traj)) {
    if(!createDisplayObjectsFromStructure(structure, keep_poses)) {
      return false;
    }
    if(!createWaypointsFromStructure(structure, keep_poses)) {
      return false;
    }
  } else {
    ROS_ERROR("AffordanceTemplate::createFromStructure() -- couldn't set the current trajectory");
    return false;
  }
  ROS_INFO("AffordanceTemplate::createFromStructure() -- done creating %s", template_type_.c_str());

  return true;
}
 
bool AffordanceTemplate::createDisplayObjectsFromStructure(affordance_template_object::AffordanceTemplateStructure structure, bool keep_poses) {

  int idx = 0;
  key_ = structure.name;

  {
    geometry_msgs::PoseStamped ps;
    ps.pose = robot_interface_->getRobotConfig().root_offset;
    ps.header.frame_id = robot_interface_->getRobotConfig().frame_id;
    frame_store_[key_] = FrameInfo(key_, ps);
  }

  for(auto &obj: structure.display_objects) {

    ROS_INFO("AffordanceTemplate::createDisplayObjectsFromStructure() creating Display Object: %s", obj.name.c_str());

    setupObjectMenu(structure, obj);

    root_frame_ = template_type_;
    if(obj.parent != "") {
      root_frame_ = obj.parent;
    } else {
      root_frame_ = key_;
    }

    // set scale factor if not already set
    if(object_scale_factor_.find(obj.name) == std::end(object_scale_factor_)) {
      object_scale_factor_[obj.name] = 1.0;
      ee_scale_factor_[obj.name] = 1.0;
      ROS_DEBUG("[AffordanceTemplate::createDisplayObjectsFromStructure] setting scale factor for %s to default 1.0", obj.name.c_str());
    } //TODO deubg??
    // else {
    //   ROS_WARN("huh %s", obj.name.c_str());
    // }

    //   object_scale_factor_[obj.name] = 1.0;
    //   ee_scale_factor_[obj.name] = 1.0;

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = root_frame_;
    int_marker.header.stamp = ros::Time(0);
    int_marker.name = obj.name;
    int_marker.description = obj.name;
    int_marker.scale = obj.controls.scale*object_scale_factor_[obj.name];

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;    

    visualization_msgs::Marker marker;
    marker.ns = obj.name;
    marker.id = idx++;
  
    if(!keep_poses || !hasObjectFrame(obj.name) ) {
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = root_frame_;
      ps.pose = originToPoseMsg(obj.origin);
      frame_store_[obj.name] = FrameInfo(obj.name, ps);
      int_marker.pose = ps.pose;
    } else {
      int_marker.pose = frame_store_[obj.name].second.pose;
      if(obj.parent != "") {
        // what is this? confused.... -SH
        int_marker.pose.position.x /= object_scale_factor_[obj.name];
        int_marker.pose.position.y /= object_scale_factor_[obj.name];
        int_marker.pose.position.z /= object_scale_factor_[obj.name];
      }
    }

    // adjust for scale factor of parent object
    if(obj.parent != "") {
      int_marker.pose.position.x *= object_scale_factor_[obj.name];
      int_marker.pose.position.y *= object_scale_factor_[obj.name];
      int_marker.pose.position.z *= object_scale_factor_[obj.name];
    }
    frame_store_[obj.name].second.pose = int_marker.pose;

    if(obj.shape.type == "mesh") {
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource = obj.shape.mesh;
      marker.scale.x = obj.shape.size[0]*object_scale_factor_[obj.name];
      marker.scale.y = obj.shape.size[1]*object_scale_factor_[obj.name];
      marker.scale.z = obj.shape.size[2]*object_scale_factor_[obj.name];
      ROS_DEBUG("Drawing Mesh for object %s : %s (scale=%.3f)", obj.name.c_str(), marker.mesh_resource.c_str(), object_scale_factor_[obj.name]);
    } else if(obj.shape.type == "box") {
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = obj.shape.size[0]*object_scale_factor_[obj.name];
      marker.scale.y = obj.shape.size[1]*object_scale_factor_[obj.name];
      marker.scale.z = obj.shape.size[2]*object_scale_factor_[obj.name];
    } else if(obj.shape.type == "sphere") {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = obj.shape.size[0]*object_scale_factor_[obj.name];
      marker.scale.y = obj.shape.size[1]*object_scale_factor_[obj.name];
      marker.scale.z = obj.shape.size[2]*object_scale_factor_[obj.name];
    } else if(obj.shape.type == "cylinder") {
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.scale.x = obj.shape.radius*object_scale_factor_[obj.name];
      marker.scale.y = obj.shape.length*object_scale_factor_[obj.name];      
    }

    control.markers.push_back(marker);

    if(obj.shape.type != "mesh") {
      control.markers[0].color.r = obj.shape.rgba[0];
      control.markers[0].color.g = obj.shape.rgba[1];
      control.markers[0].color.b = obj.shape.rgba[2];
      control.markers[0].color.a = obj.shape.rgba[3];
    } else {
      control.markers[0].mesh_use_embedded_materials = true;
      control.markers[0].color.r = 1.0;
      control.markers[0].color.g = 1.0;
      control.markers[0].color.b = 1.0;
      control.markers[0].color.a = 1.0;
    }

    // double scale = 1.0;
    // if obj in self.object_controls :
    //     scale = self.object_controls[obj]['scale']*self.object_scale_factor[obj]

    //  # int_marker = CreateInteractiveMarker(self.frame_id, obj.name, scale)
    int_marker.controls.push_back(control);
    if(object_controls_display_on_) {
      std::vector<visualization_msgs::InteractiveMarkerControl> dof_controls;
      dof_controls = utils::MarkerHelper::makeCustomDOFControls(obj.controls.translation[0], 
                                                                obj.controls.translation[1], 
                                                                obj.controls.translation[2],
                                                                obj.controls.rotation[0],
                                                                obj.controls.rotation[1],
                                                                obj.controls.rotation[2]);

      for (auto &c: dof_controls) {
        int_marker.controls.push_back(c);
      }
    }   

    //self.marker_pose_offset[obj] = self.pose_from_origin(self.object_origin[obj])

    addInteractiveMarker(int_marker);

    MenuHandleKey key;
    key[obj.name] = {"Hide Controls"};
    if(object_controls_display_on_) {
      marker_menus_[obj.name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
    } else {
      marker_menus_[obj.name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
    }

    marker_menus_[obj.name].apply( *server_, obj.name );
  }
  server_->applyChanges();

  return true;
}


bool AffordanceTemplate::createWaypointsFromStructure(affordance_template_object::AffordanceTemplateStructure structure, bool keep_poses) 
{
  // DEBUG statements -- TODO
  ROS_INFO("AffordanceTemplate::createWaypointsFromStructure() -- trajectory: %s", current_trajectory_.c_str());

  int wp_ids = 0;

  Trajectory traj;
  if(!getTrajectory(structure.ee_trajectories, current_trajectory_, traj)){
    ROS_ERROR("AffordanceTemplate::createWaypointsFromStructure() -- couldn't get the request trajectory");
    return false;
  }

  for(auto &wp_list: traj.ee_waypoint_list) {

    int ee_id = wp_list.id;
    std::map<int, std::string> ee_name_map = robot_interface_->getEENameMap();
    std::string ee_name = ee_name_map[ee_id];

    int wp_id = 0;

    ROS_INFO("AffordanceTemplate::createWaypointsFromStructure() creating Trajectory for Waypoint[%d]: %s", ee_id, ee_name.c_str());

    setTrajectoryFlags(traj);

    for(auto &wp: wp_list.waypoints) {

      std::string wp_name = createWaypointID(ee_id, wp_id);
      ROS_INFO("AffordanceTemplate::createWaypointsFromStructure() creating Waypoint: %s", wp_name.c_str());

      setupWaypointMenu(structure, wp_name);
      geometry_msgs::Pose display_pose = originToPoseMsg(wp.origin);  

      std::string parent_obj = appendID(wp.display_object);
      double parent_scale = object_scale_factor_[parent_obj]*ee_scale_factor_[parent_obj];  

      display_pose.position.x *= parent_scale;
      display_pose.position.y *= parent_scale;
      display_pose.position.z *= parent_scale;

      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = parent_obj;
      int_marker.header.stamp = ros::Time(0);
      int_marker.name = wp_name;
      int_marker.description = wp_name;
      int_marker.scale = wp.controls.scale;

    
      if(!keep_poses || !hasWaypointFrame(wp_name) ) {
        //store base WP pose to publish by TF
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = parent_obj;
        ps.pose = display_pose;
        frame_store_[wp_name] = FrameInfo(wp_name, ps);
        int_marker.pose = ps.pose;
      } else {
        int_marker.pose = frame_store_[wp_name].second.pose;
        // if(parent_obj != "") {
        //   int_marker.pose.position.x /= object_scale_factor_[obj.name];
        //   int_marker.pose.position.y /= object_scale_factor_[obj.name];
        //   int_marker.pose.position.z /= object_scale_factor_[obj.name];
        // }
      }

      visualization_msgs::InteractiveMarkerControl menu_control;
      menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;    
      menu_control.always_visible = true;
      
      MenuHandleKey key;
      if(waypoint_flags_[current_trajectory_].run_backwards) {
        key[wp_name] = {"Compute Backwards Path"};
        marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
      }  
      if(waypoint_flags_[current_trajectory_].auto_execute) {
        key[wp_name] = {"Execute On Move"};
        marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
      }  
      if(waypoint_flags_[current_trajectory_].loop) {
        key[wp_name] = {"Loop Path"};
        marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
      }  
      if(waypoint_flags_[current_trajectory_].controls_on[wp_name]) {
        key[wp_name] = {"Hide Controls"};
        marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
      }  else {
        key[wp_name] = {"Hide Controls"};
        marker_menus_[wp_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
      }

      std::string ee_pose_name;;
      try {
        ee_pose_name = robot_interface_->getEEPoseNameMap(ee_name)[wp.ee_pose];
      } catch(...) {
        ee_pose_name = "current";
      }
      ROS_INFO("AffordanceTemplate::createWaypointsFromStructure()   ee_pose_name: %s", ee_pose_name.c_str());
               
      visualization_msgs::MarkerArray markers;
      end_effector_helper::EndEffectorHelperConstPtr ee_link_data;
      if(robot_interface_->getEELinkData(ee_name, ee_link_data)) { 
        if(!ee_link_data->getMarkersForPose(ee_pose_name, markers)) {
          ROS_ERROR("AffordanceTemplate::createWaypointsFromStructure() -- problem getting pose markers for EE %s, pose: %s", ee_name.c_str(), ee_pose_name.c_str());
          return false;
        } 
      } else {
        ROS_ERROR("AffordanceTemplate::createWaypointsFromStructure() -- no link data for EE %s", ee_name.c_str());
        return false;        
      }

      tf::Transform wpTee, eeTtf;

      // std::cout << "getManipulatorOffsetPose() =" << std::endl;       
      // std::cout << robot_interface_->getManipulatorOffsetPose(ee_name) << std::endl;
      // std::cout << "getToolOffsetPose() =" << std::endl;       
      // std::cout << robot_interface_->getToolOffsetPose(ee_name) << std::endl;

      geometry_msgs::PoseStamped ee_ps;
      ee_ps.header.frame_id = wp_name;
      ee_ps.pose = robot_interface_->getManipulatorOffsetPose(ee_name);
      std::string ee_frame_name = wp_name + "/ee";
      frame_store_[ee_frame_name] = FrameInfo(ee_frame_name, ee_ps);

      geometry_msgs::PoseStamped tf_ps;
      tf_ps.header.frame_id = ee_frame_name;
      tf_ps.pose = robot_interface_->getToolOffsetPose(ee_name);
      std::string tf_frame_name = wp_name + "/tf";
      frame_store_[tf_frame_name] = FrameInfo(tf_frame_name, tf_ps);
    
      try {
        tf::poseMsgToTF(robot_interface_->getManipulatorOffsetPose(ee_name),wpTee);
        tf::poseMsgToTF(robot_interface_->getToolOffsetPose(ee_name),eeTtf);
      } catch(...) {
        ROS_ERROR("AffordanceTemplate::createWaypointsFromStructure() -- error getting transforms for %s", ee_name.c_str());
      }

      for(auto &m: markers.markers) {
        visualization_msgs::Marker ee_m = m;
        ee_m.header.frame_id = tf_frame_name;
        ee_m.ns = name_;
        ee_m.pose = m.pose;
        menu_control.markers.push_back( ee_m );
      }
      // scale = 1.0
      // if wp in self.waypoint_controls[trajectory] :
      //     scale = self.waypoint_controls[trajectory][wp]['scale']


      int_marker.controls.push_back(menu_control);
      if(waypoint_flags_[current_trajectory_].controls_on[wp_name]) {
        std::vector<visualization_msgs::InteractiveMarkerControl> dof_controls;
        dof_controls = utils::MarkerHelper::makeCustomDOFControls(wp.controls.translation[0], 
                                                                  wp.controls.translation[1], 
                                                                  wp.controls.translation[2],
                                                                  wp.controls.rotation[0],
                                                                  wp.controls.rotation[1],
                                                                  wp.controls.rotation[2]);

        for (auto &c: dof_controls) {
          int_marker.controls.push_back(c);
        }
      }

      // std::cout << int_marker << std::endl;
      addInteractiveMarker(int_marker);

      marker_menus_[wp_name].apply( *server_, wp_name );
      server_->applyChanges();          
 
      wp_id++;
    }
  }
  ROS_INFO("AffordanceTemplate::createWaypointsFromStructure() -- done");
 
  return true;
}


void AffordanceTemplate::setupObjectMenu(AffordanceTemplateStructure structure, DisplayObject obj)
{
  for(auto& o : object_menu_options_) 
  {
    if(o.first == "Choose Trajectory") 
      setupTrajectoryMenu(structure, obj.name);
    else if (o.first.find("Add Waypoint") != std::string::npos)
      setupAddWaypointMenuItem(structure, obj.name, o.first);
    else 
      setupSimpleMenuItem(structure, obj.name, o.first, o.second);
  }
}

void AffordanceTemplate::setupWaypointMenu(AffordanceTemplateStructure structure, std::string name)
{
  for(auto& o : waypoint_menu_options_) 
  {
    if(o.first == "Change End-Effector Pose")
      setupEndEffectorPoseMenu(name);
    else
      setupSimpleMenuItem(structure, name, o.first, o.second);
  }
}

void AffordanceTemplate::setupSimpleMenuItem(AffordanceTemplateStructure structure, const std::string& name, const std::string& menu_text, bool has_check_box)
{
  MenuHandleKey key;
  key[name] = {menu_text};  
  group_menu_handles_[key] = marker_menus_[name].insert( menu_text, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) );  
  if(has_check_box) {
    marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
}

void AffordanceTemplate::setupAddWaypointMenuItem(AffordanceTemplateStructure structure, std::string name, std::string menu_text)
{
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[name].insert( menu_text);

  for(auto& ee: robot_interface_->getEENameMap()) 
  {
    std::string ee_readable = robot_interface_->getReadableEEName(ee.second);

    MenuHandleKey key;
    key[name] = {menu_text, ee.second};
    group_menu_handles_[key] = marker_menus_[name].insert( sub_menu_handle, ee_readable, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) );   
    
    ROS_DEBUG("[AffordanceTemplate::setupAddWaypointMenuItem] adding submenu text %s to menu item %s", ee_readable.c_str(), menu_text.c_str());
  }
}

void AffordanceTemplate::setupTrajectoryMenu(AffordanceTemplateStructure structure, const std::string& name)
{
  std::string menu_text = "Choose Trajectory";
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[name].insert( menu_text );
  for(auto &traj: structure.ee_trajectories) 
  {
    MenuHandleKey key;
    key[name] = {menu_text, traj.name};
    group_menu_handles_[key] = marker_menus_[name].insert( sub_menu_handle, traj.name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) );   
  
    if(traj.name == current_trajectory_) 
      marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
    else
      marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );       
    
    ROS_DEBUG("[AffordanceTemplate::setupTrajectoryMenu] adding submenu text %s to menu item %s", traj.name.c_str(), menu_text.c_str());
  }
}

void AffordanceTemplate::setupEndEffectorPoseMenu(const std::string& name)
{
  std::string menu_text = "Change End-Effector Pose";
  int ee_id = getEEIDfromWaypointName(name);
  std::string ee_name = robot_interface_->getEEName(ee_id);
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[name].insert( menu_text );
  for(auto &pose_name: robot_interface_->getEEPoseNames(ee_name)) {
    MenuHandleKey key;
    key[name] = {menu_text, pose_name};
    group_menu_handles_[key] = marker_menus_[name].insert( sub_menu_handle, pose_name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) ); 
  }
}

int AffordanceTemplate::getEEIDfromWaypointName(const std::string wp_name) 
{
  std::string delimiter = ".";
  size_t pos = 0;
  std::string token;
  if ((pos = wp_name.find(delimiter)) != std::string::npos) {
    token = wp_name.substr(0, pos);
    return std::stoi(token);
  }
  ROS_ERROR("AffordanceTemplate::getEEIDfromWaypointName() -- could not find EE ID from %s", wp_name.c_str());
  return -1;
}

void AffordanceTemplate::addInteractiveMarker(visualization_msgs::InteractiveMarker m)
{
  ROS_INFO("AffordanceTemplate::addInteractiveMarker() -- %s with frame: %s", m.name.c_str(), m.header.frame_id.c_str());
  std::string name = m.name;
  int_markers_[m.name] = m;
  server_->insert(m);
  server_->setCallback(m.name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ));
}

void AffordanceTemplate::removeInteractiveMarker(std::string marker_name) 
{
  ROS_INFO("[AffordanceTemplate::removeInteractiveMarker] removing marker %s", marker_name.c_str());
  server_->erase(marker_name);
  server_->applyChanges();
}

void AffordanceTemplate::removeAllMarkers() 
{
  for(auto &m: int_markers_)
    removeInteractiveMarker(m.first);
  group_menu_handles_.clear();
  int_markers_.clear();
  marker_menus_.clear();
  server_->applyChanges();
}

void AffordanceTemplate::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) 
{
  // ROS_INFO("AffordanceTemplate::processFeedback() -- %s", feedback->marker_name.c_str());

  interactive_markers::MenuHandler::CheckState state;

  // set up key maps for easy comparison to menu handler ID
  MenuHandleKey wp_before_key;
  MenuHandleKey wp_after_key;
  MenuHandleKey reset_key;
  MenuHandleKey save_key;
  MenuHandleKey delete_key;
  MenuHandleKey hide_controls_key;
  MenuHandleKey plan_test_key;
  MenuHandleKey execute_test_key;
  
  wp_before_key[feedback->marker_name] = {"Add Waypoint Before"};
  wp_after_key[feedback->marker_name] = {"Add Waypoint After"};
  reset_key[feedback->marker_name] = {"Reset"};
  save_key[feedback->marker_name] = {"Save"};
  delete_key[feedback->marker_name] = {"Delete Waypoint"};
  plan_test_key[feedback->marker_name] = {"Plan Test"};
  execute_test_key[feedback->marker_name] = {"Execute Test"};
  hide_controls_key[feedback->marker_name] = {"Hide Controls"};

  if(hasObjectFrame(feedback->marker_name) || hasWaypointFrame(feedback->marker_name)) {
    geometry_msgs::Pose p = feedback->pose;
    if(feedback->header.frame_id != frame_store_[feedback->marker_name].second.header.frame_id) {
      geometry_msgs::PoseStamped ps;
      ps.pose = feedback->pose;
      ps.header = feedback->header;
      tf_listener_.transformPose (frame_store_[feedback->marker_name].second.header.frame_id, ps, ps);
      p = ps.pose;
    }
    frame_store_[feedback->marker_name].second.pose = p;
  }
  
  switch ( feedback->event_type ) {

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP :
      ROS_INFO("[AffordanceTemplate::processFeedback] %s mouse up", feedback->marker_name.c_str());
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT : {
      ROS_INFO("[AffordanceTemplate::processFeedback] %s selected menu entry: %d", feedback->marker_name.c_str(), feedback->menu_entry_id);
      // ROS_DEBUG("AffordanceTemplate::processFeedback() -- MENU_SELECT");
      // ROS_DEBUG("AffordanceTemplate::processFeedback() --   menu id: %d", feedback->menu_entry_id);
      // ROS_DEBUG("AffordanceTemplate::processFeedback() --   pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s", 
      //                                                             feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z, 
      //                                                             feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w,
      //                                                             feedback->header.frame_id.c_str());

      // *****
      // FIXME : this may not be the best way to figure out if we're dealing with the object or an EE
      // *****

      // 
      // check for 'Add Waypoint Before' for EE objects
      if (group_menu_handles_.find(wp_before_key) != std::end(group_menu_handles_)) 
      {
        if (group_menu_handles_[wp_before_key] == feedback->menu_entry_id)
        {
          // Trajectory traj;
          // if (getTrajectory(structure_.ee_trajectories, current_trajectory_, traj)) // FIXME - why wouldn't this give us the actual reference we want to alter data? everything is setup to use references but this seems to give us a copy instead
          bool found = false;
          for (auto& traj : structure_.ee_trajectories)
          {
            if (traj.name == current_trajectory_)
            {
              // look for the object the user selected in our waypoint list
              for (auto& wp_list: traj.ee_waypoint_list) 
              {
                int wp_id = -1; // init to -1 because we pre-add
                for (auto& wp: wp_list.waypoints) 
                {
                  std::string wp_name = createWaypointID(wp_list.id, ++wp_id);
                  if (wp_name == feedback->marker_name)
                  {
                    ROS_DEBUG("[AffordanceTemplate::processFeedback::Add Waypoint Before] for EE waypoint %s", feedback->marker_name.c_str());
                    // TODO and FIXME - this is all just testing insertion
                    affordance_template_object::EndEffectorWaypoint eewp;
                    eewp.ee_pose = 1;
                    eewp.display_object = "test_before_object";
                    eewp.origin.position[0] = eewp.origin.position[1] = eewp.origin.position[2] = 1.0;
                    eewp.origin.orientation[0] = eewp.origin.orientation[1] = eewp.origin.orientation[2] = 0.0;
                    eewp.controls.translation[0] = eewp.controls.translation[1] = eewp.controls.translation[2] = true;
                    eewp.controls.rotation[0] = eewp.controls.rotation[1] = eewp.controls.rotation[2] = true;
                    eewp.controls.scale = 1.0;
                    wp_list.waypoints.insert(wp_list.waypoints.begin(), eewp);

                    found = true;
                    break;
                  }
                }

                if (found) // already found the object - no reason to continue the for loop
                  break;
              }
            }

            if (found) // already found the object - no reason to continue the for loop
              break;
          }
        }
      }

      // 
      // check for 'Add Waypoint Before' for AT object (wheel, door, etc) 
      for (auto& ee: robot_interface_->getEENameMap()) 
      {
        bool found = false;  
        MenuHandleKey key;
        key[feedback->marker_name] = {"Add Waypoint Before", ee.second};
        if (group_menu_handles_.find(key) != std::end(group_menu_handles_)) 
        {
          if (group_menu_handles_[key] == feedback->menu_entry_id)
          {
            // Trajectory traj; 
            // if (getTrajectory(structure_.ee_trajectories, current_trajectory_, traj)) // FIXME - why wouldn't this give us the actual reference we want to alter data? everything is setup to use references but this seems to give us a copy instead
            for (auto& traj : structure_.ee_trajectories)
            {
              if (traj.name == current_trajectory_)
              {
                for (auto& wp_list : traj.ee_waypoint_list) // go through our list of EE waypoints - match based on EE ID
                {
                  if (wp_list.id == robot_interface_->getEEID(ee.second))
                  {
                    ROS_DEBUG("[AffordanceTemplate::processFeedback::Add Waypoint Before] for trajectory: %s and end effector: %s", current_trajectory_.c_str(), robot_interface_->getReadableEEName(ee.second).c_str());
                    // TODO and FIXME - this is all just testing insertion
                    affordance_template_object::EndEffectorWaypoint wp;
                    wp.ee_pose = 1; 
                    wp.display_object = "test_before_object";
                    wp.origin.position[0] = wp.origin.position[1] = wp.origin.position[2] = 1.0;
                    wp.origin.orientation[0] = wp.origin.orientation[1] = wp.origin.orientation[2] = 0.0;
                    wp.controls.translation[0] = wp.controls.translation[1] = wp.controls.translation[2] = true;
                    wp.controls.rotation[0] = wp.controls.rotation[1] = wp.controls.rotation[2] = true;
                    wp.controls.scale = 1.0;
                    wp_list.waypoints.insert(wp_list.waypoints.begin(), wp);

                    found = true;
                    break;
                  }
                }

                if (found) // already found the object - no reason to continue the for loop
                  break;
              }
            }
          }
        }

        if (found) // already found the object - no reason to continue the for loop
          break;
      }

      // 
      // check for 'Add Waypoint After' for EE objects
      if (group_menu_handles_.find(wp_after_key) != std::end(group_menu_handles_)) 
      {
        if (group_menu_handles_[wp_after_key] == feedback->menu_entry_id)
        {
          // Trajectory traj;
          // if (getTrajectory(structure_.ee_trajectories, current_trajectory_, traj)) // FIXME - why wouldn't this give us the actual reference we want to alter data? everything is setup to use references but this seems to give us a copy instead
          bool found = false;
          for (auto& traj : structure_.ee_trajectories)
          {
            if (traj.name == current_trajectory_)
            {
              // look for the object the user selected in our waypoint list
              for (auto& wp_list: traj.ee_waypoint_list) 
              {
                int wp_id = -1; // init to -1 because we pre-add
                for (auto& wp: wp_list.waypoints) 
                {
                  std::string wp_name = createWaypointID(wp_list.id, ++wp_id);
                  if (wp_name == feedback->marker_name)
                  {
                    ROS_DEBUG("[AffordanceTemplate::processFeedback::Add Waypoint After] for EE waypoint %s", feedback->marker_name.c_str());
                    // TODO and FIXME - this is all just testing insertion
                    affordance_template_object::EndEffectorWaypoint eewp;
                    eewp.ee_pose = 1;
                    eewp.display_object = "test_after_object";
                    eewp.origin.position[0] = eewp.origin.position[1] = eewp.origin.position[2] = 1.0;
                    eewp.origin.orientation[0] = eewp.origin.orientation[1] = eewp.origin.orientation[2] = 0.0;
                    eewp.controls.translation[0] = eewp.controls.translation[1] = eewp.controls.translation[2] = true;
                    eewp.controls.rotation[0] = eewp.controls.rotation[1] = eewp.controls.rotation[2] = true;
                    eewp.controls.scale = 1.0;
                    wp_list.waypoints.push_back(eewp);

                    found = true;
                    break;
                  }
                }

                if (found) // already found the object - no reason to continue the for loop
                  break;
              }
            }

            if (found) // already found the object - no reason to continue the for loop
              break;
          }
        }
      }

      // 
      // check for 'Add Waypoint After' for AT object (wheel, door, etc) 
      for (auto& ee: robot_interface_->getEENameMap()) 
      {
        bool found = false;  
        MenuHandleKey key;
        key[feedback->marker_name] = {"Add Waypoint After", ee.second};
        if (group_menu_handles_.find(key) != std::end(group_menu_handles_)) 
        {
          if (group_menu_handles_[key] == feedback->menu_entry_id)
          {
            // Trajectory traj; 
            // if (getTrajectory(structure_.ee_trajectories, current_trajectory_, traj)) // FIXME - why wouldn't this give us the actual reference we want to alter data? everything is setup to use references but this seems to give us a copy instead
            for (auto& traj : structure_.ee_trajectories)
            {
              if (traj.name == current_trajectory_)
              {
                for (auto& wp_list : traj.ee_waypoint_list) // go through our list of EE waypoints - match based on EE ID
                {
                  if (wp_list.id == robot_interface_->getEEID(ee.second))
                  {
                    ROS_DEBUG("[AffordanceTemplate::processFeedback::Add Waypoint After] for trajectory: %s and end effector: %s", current_trajectory_.c_str(), robot_interface_->getReadableEEName(ee.second).c_str());
                    // TODO and FIXME - this is all just testing insertion
                    affordance_template_object::EndEffectorWaypoint wp;
                    wp.ee_pose = 1; 
                    wp.display_object = "test_after_object";
                    wp.origin.position[0] = wp.origin.position[1] = wp.origin.position[2] = 1.0;
                    wp.origin.orientation[0] = wp.origin.orientation[1] = wp.origin.orientation[2] = 0.0;
                    wp.controls.translation[0] = wp.controls.translation[1] = wp.controls.translation[2] = true;
                    wp.controls.rotation[0] = wp.controls.rotation[1] = wp.controls.rotation[2] = true;
                    wp.controls.scale = 1.0;
                    wp_list.waypoints.push_back(wp);

                    found = true;
                    break;
                  }
                }

                if (found) // already found the object - no reason to continue the for loop
                  break;
              }
            }
          }
        }

        if (found) // already found the object - no reason to continue the for loop
          break;
      }

      // 
      // delete waypoint
      if (group_menu_handles_.find(delete_key) != group_menu_handles_.end())
      {
        if (group_menu_handles_[delete_key] == feedback->menu_entry_id)
        {
          bool found = false;
          ROS_DEBUG("[AffordanceTemplate::processFeedback::Delete Waypoint] deleting waypoint: %s", feedback->marker_name.c_str());
          for (auto& traj : structure_.ee_trajectories)
          {
            if (traj.name == current_trajectory_)
            {
              // look for the object the user selected in our waypoint list
              for (auto& wp_list: traj.ee_waypoint_list) 
              {
                int wp_id = -1; // init to -1 because we pre-add
                for (auto& wp: wp_list.waypoints) 
                {
                  std::string wp_name = createWaypointID(wp_list.id, ++wp_id);
                  if (wp_name == feedback->marker_name)
                  {
                    if (wp_list.waypoints.size() == 1)
                      wp_list.waypoints.clear();
                    else
                      wp_list.waypoints.erase(wp_list.waypoints.begin() + wp_id);

                    found = true;

                    //FIXME:: is this the best way to handle methodsl ike these?? 
                    //        should these be called at the end of the processFeedback
                    //        or should we be using server->apply() instead??
                    removeAllMarkers();
                    createFromStructure(structure_, false, current_trajectory_); 
                    break;
                  }
                }
                if (found)
                  break;
              }
            }
            if (found)
              break;
          }
        }
      }

      if(group_menu_handles_.find(reset_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[reset_key] == feedback->menu_entry_id) {
          ROS_INFO("AffordanceTemplate::processFeedback::Reset] resetting current structure to the inital structure.");
          structure_ = initial_structure_;      
          appendIDToStructure(structure_);
          removeAllMarkers();
          clearTrajectoryFlags();
          createFromStructure(structure_, false, current_trajectory_); //FIXME:: make sure current_traj is i ORIGINAL list of traj
        }
      }

      if (group_menu_handles_.find(save_key) != group_menu_handles_.end()) 
      {
        if (group_menu_handles_[save_key] == feedback->menu_entry_id) 
        {
          ROS_DEBUG("[AffordanceTemplate::processFeedback::Save] saving file");
          std::vector<std::string> keys;
          boost::split(keys, structure_.filename, boost::is_any_of("/"));
          if (keys.size())
            saveToDisk(keys.back(), structure_.image, feedback->marker_name, true);
          else
            ROS_ERROR("[AffordanceTemplate::processFeedback::Save] invalid filename: %s", structure_.filename.c_str());
        }
      }

      if(group_menu_handles_.find(hide_controls_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[hide_controls_key] == feedback->menu_entry_id) {
          ROS_INFO("AffordanceTemplate::processFeedback() --   CONTROLS TOGGLE");
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
              if(isObject(feedback->marker_name)) {
                object_controls_display_on_ = true;
              } else {
                waypoint_flags_[current_trajectory_].controls_on[feedback->marker_name] = true;
              }
            } else {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED );
              if(isObject(feedback->marker_name)) {
                object_controls_display_on_ = false;
              } else {
                waypoint_flags_[current_trajectory_].controls_on[feedback->marker_name] = false;
              }
            }
          }
          removeAllMarkers();
          createFromStructure(structure_, true, current_trajectory_);
        }
      }
      
      if(group_menu_handles_.find(plan_test_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[plan_test_key] == feedback->menu_entry_id) {
          ROS_WARN("AffordanceTemplate::processFeedback() --   PLAN");

          std::vector<std::string> ee_names = {"left_hand"};
          planPathToWaypoints(ee_names, 1, false, false); 

        }
      }

      if(group_menu_handles_.find(execute_test_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[execute_test_key] == feedback->menu_entry_id) {
          ROS_WARN("AffordanceTemplate::processFeedback() --   EXECUTE");

          std::vector<std::string> ee_names = {"left_hand"};
          moveToWaypoints(ee_names); 

        }
      }
      
      //
      // switch trajectories using the context menu
      for (auto &traj: structure_.ee_trajectories) 
      {
        MenuHandleKey key;
        key[feedback->marker_name] = {"Choose Trajectory", traj.name}; // FIXME -- can this be static like this??
        if (group_menu_handles_.find(key) != group_menu_handles_.end())
        {
          marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED);
          if (group_menu_handles_[key] == feedback->menu_entry_id) 
          {
            ROS_DEBUG("[AffordanceTemplate::processFeedback::Choose Trajectory] found matching trajectory name %s", traj.name.c_str());
            setTrajectory(traj.name);
            marker_menus_[feedback->marker_name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED);
          }
        }
      }

      break;
    }
    default : 
      //ROS_WARN("[AffordanceTemplate::processFeedback] got unrecognized or unmatched menu event: %d", feedback->event_type);
      break;
  }
  server_->applyChanges();
  marker_menus_[feedback->marker_name].apply( *server_, feedback->marker_name );
}

bool AffordanceTemplate::isObject(const std::string& obj) {
  for(auto &o : structure_.display_objects) {
    if(o.name == obj) {
      return true;
    }
  }
  return false;
}

bool AffordanceTemplate::isWaypoint(const std::string& wp) {
  return !isObject(wp);
}


bool AffordanceTemplate::hasObjectFrame(std::string obj) {
  return isObject(obj) && (frame_store_.find(obj) != std::end(frame_store_)); 
}

bool AffordanceTemplate::hasWaypointFrame(std::string wp) {
  return isWaypoint(wp) && (frame_store_.find(wp) != std::end(frame_store_)); 
}

int AffordanceTemplate::getNumWaypoints(const AffordanceTemplateStructure structure, const std::string traj_name, const int ee_id) {
  for(auto &traj : structure.ee_trajectories) {
    if(traj.name == traj_name) {
      for(auto &ee_list : traj.ee_waypoint_list) {
        if(ee_list.id == ee_id) {
          return ee_list.waypoints.size();
        }
      }
    }
  }
  return 0;
}


bool AffordanceTemplate::computePathSequence(AffordanceTemplateStructure structure, std::string traj_name, int ee_id, int idx, int steps, bool backwards, std::vector<int> &sequence_ids, int &next_path_idx)
{ 
  sequence_ids.clear();
  if (steps == 0) {
    sequence_ids.push_back(idx);
    next_path_idx = idx;
    return true;
  } else {
    int max_idx = getNumWaypoints(structure, traj_name, ee_id)-1;
    int cap = max_idx+1;
    int inc = 1;
    if(backwards) {
      inc = -1;
    }
    if(idx == -1) {
      if(backwards) {
        sequence_ids.push_back(max_idx);
        idx = max_idx;
      } else {
        sequence_ids.push_back(0);
        idx = 0;
      }
      steps--;
    }
    for(int s=0; s<steps; s++) {
      idx += inc;
      if(idx == -1) {
        if(backwards) {
          idx = max_idx;
        } else {
          idx = 0;
        }
      } else {
        idx = idx%cap;
      }
      sequence_ids.push_back(idx);      
    }
    next_path_idx = sequence_ids.back();
  }
  return true;
}

 // list of ee names, steps, direct, backwards; return map of bools keyed on EE name
std::map<std::string, bool> AffordanceTemplate::planPathToWaypoints(const std::vector<std::string>& ee_names, int steps, bool direct, bool backwards)
{

  ROS_INFO("AffordanceTemplate::planPathToWaypoints()");

  std::map<std::string, bool> ret;
  std::map<std::string, std::vector<geometry_msgs::PoseStamped> > goals;

  for(auto ee: ee_names) {
    ret[ee] = false;
 
    int ee_id = robot_interface_->getEEID(ee);
    int max_idx = getNumWaypoints(structure_, current_trajectory_, ee_id);
    std::string manipulator_name = robot_interface_->getManipulator(ee);

    plan_status_[current_trajectory_][ee].backwards = backwards;
    plan_status_[current_trajectory_][ee].direct = direct;
    plan_status_[current_trajectory_][ee].plan_valid = false;
    plan_status_[current_trajectory_][ee].exec_valid = false;
    plan_status_[current_trajectory_][ee].sequence_ids.clear();
    plan_status_[current_trajectory_][ee].sequence_poses.clear();

    int current_idx = plan_status_[current_trajectory_][ee].current_idx;

    ROS_INFO("AffordanceTemplate::planPathToWaypoints() -- configuring plan goal for %s[%d]. manipulator=%s, size=%d", ee.c_str(), ee_id,manipulator_name.c_str(), max_idx);

    std::vector<int> sequence_ids;
    int next_path_idx;
    if(computePathSequence(structure_, current_trajectory_, ee_id, current_idx, steps, backwards, sequence_ids, next_path_idx)) {    
      ROS_INFO("AffordanceTemplate::planPathToWaypoints() -- got path sequence");
      std::cout << "path sequence: [ ";
      for (auto i: sequence_ids) {
        std::cout << i << " ";
      }
      std::cout << "]" << std::endl;      
    } else {
      ROS_ERROR("AffordanceTemplate::planPathToWaypoints() -- failed to get path sequence!!");
      return ret;
    }
    if(direct) {
      sequence_ids.clear();
      sequence_ids.push_back(next_path_idx);
      ROS_INFO("AffordanceTemplate::planPathToWaypoints() -- moving direct");
    }

    plan_status_[current_trajectory_][ee].sequence_ids = sequence_ids;
    plan_status_[current_trajectory_][ee].goal_idx = next_path_idx;

    std::string next_path_str = createWaypointID(ee_id, next_path_idx);
    if(direct) {
      ROS_INFO("AffordanceTemplate::planPathToWaypoints() -- computing DIRECT path to waypoint[%d]: %s", next_path_idx, next_path_str.c_str());
    } else {
      ROS_INFO("AffordanceTemplate::planPathToWaypoints() -- computing path to waypoint[%d]: %s", next_path_idx, next_path_str.c_str());
    }

    goals[manipulator_name].clear();
    for(auto &idx : plan_status_[current_trajectory_][ee].sequence_ids) {
      next_path_str = createWaypointID(ee_id, idx);
      ROS_INFO("AffordanceTemplate::planPathToWaypoints() --   next goal: %s", next_path_str.c_str());
      geometry_msgs::PoseStamped pt = frame_store_[next_path_str + "/tf"].second;    
      goals[manipulator_name].push_back(pt);
      plan_status_[current_trajectory_][ee].sequence_poses.push_back(pt);
    }
  
  }
  // if(robot_interface_->getPlanner()->planPaths(goals, false, true)) {
  if(robot_interface_->getPlanner()->planCartesianPaths(goals, false, true)) {
    
    ROS_INFO("AffordanceTemplate::planPathToWaypoints() -- planning succeeded");
    for(auto ee: ee_names) {
       plan_status_[current_trajectory_][ee].plan_valid = true;
       ret[ee] = true;
    }
  } else {
    ROS_WARN("AffordanceTemplate::planPathToWaypoints() -- planning failed");
  }

  return ret;
}

 // list of ee waypoints to move to, return true if all waypoints were valid
bool AffordanceTemplate::moveToWaypoints(const std::vector<std::string>& ee_names) 
{
  ROS_INFO("AffordanceTemplate::moveToWaypoints()");
  std::vector<std::string> valid_ee_plans;
  std::vector<std::string> m_names;
  for(auto ee: ee_names) {
    if (plan_status_[current_trajectory_][ee].plan_valid) {
      valid_ee_plans.push_back(ee);
      m_names.push_back(robot_interface_->getManipulator(ee));
    } else {
      plan_status_[current_trajectory_][ee].exec_valid = false;
    }
  }
  if(robot_interface_->getPlanner()->executePlans(m_names)) {
    ROS_INFO("AffordanceTemplate::moveToWaypoints() -- execution succeeded");
    for(auto ee: valid_ee_plans) {
      plan_status_[current_trajectory_][ee].current_idx = plan_status_[current_trajectory_][ee].goal_idx;
      plan_status_[current_trajectory_][ee].plan_valid = false;
      plan_status_[current_trajectory_][ee].exec_valid = true;
    }
    return true;
  }
  ROS_WARN("AffordanceTemplate::moveToWaypoints() -- execution failed");
  return false;
}

void AffordanceTemplate::run()
{
  ros::Rate loop_rate(loop_rate_);
  tf::Transform transform;
  FrameInfo fi;

  ROS_INFO("[AffordanceTemplate] spinning...");
  while(ros::ok() && running_)
  {
    for(auto &f: frame_store_) 
    {
      fi = f.second;
      tf::poseMsgToTF(fi.second.pose, transform);
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fi.second.header.frame_id, fi.first));
    }
    loop_rate.sleep();
  }
  ROS_INFO("[AffordanceTemplate] leaving spin thread. template must be shutting down...");
}

void AffordanceTemplate::stop()
{
  ROS_WARN("[AffordanceTemplate::stop] %s being asked to stop..", name_.c_str());
  running_ = false;
  removeAllMarkers();
}

bool AffordanceTemplate::setObjectScaling(const std::string& key, double scale_factor, double ee_scale_factor)
{ 
  ROS_DEBUG("[AffordanceTemplate::setObjectScaling] setting object %s scaling to %g, %g", key.c_str(), scale_factor, ee_scale_factor);
  
  // TODO this probably needs double-checking 
  //      also, this should only happen if we want to save scaling factor
  // for ( auto& d : structure_.display_objects)
  // {
  //   if ( d.name == key)
  //     d.controls.scale = scale_factor;
  // }
  // for ( auto& t : structure_.ee_trajectories)
  // {
  //   for ( auto& e : t.ee_waypoint_list)
  //   {
  //     for ( auto& w : e.waypoints)
  //       w.controls.scale = ee_scale_factor;
  //   }
  // }

  object_scale_factor_[key] = scale_factor;
  ee_scale_factor_[key] = ee_scale_factor;

  removeAllMarkers();
  return createFromStructure(structure_);
}