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
  setupMenuOptions();
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

}

bool AffordanceTemplate::loadFromFile(std::string filename, geometry_msgs::Pose pose, AffordanceTemplateStructure &structure)
{

  at_parser_.loadFromFile(filename, structure);

  // store copies in class, one as backup to reset/restore later
  initial_structure_ = structure;
  structure_ = structure;

  appendIDToStructure(structure_);
  createFromStructure(structure_);

  return true;
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

bool AffordanceTemplate::getTrajectory(TrajectoryList traj_list, std::string traj_name, Trajectory &traj) 
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

bool AffordanceTemplate::setCurrentTrajectory(TrajectoryList traj_list, std::string traj) 
{
  
  // set the default (current) traj to the input one.
  // if no input request, find the first valid one

  current_trajectory_ = "";
  if(traj!="") {
    for (auto &t: traj_list) {
      if(t.name == traj) {
        if(isValidTrajectory(t)) {
          current_trajectory_ = t.name;
          ROS_INFO("AffordanceTemplate::createFromStructure() -- setting current trajectory to: %s", current_trajectory_.c_str());
          break;
        }
      }
    }
  } 
  if(current_trajectory_=="") {
    for (auto &t: traj_list) {
      if(isValidTrajectory(t)) {
        current_trajectory_ = t.name;
        if(traj!=current_trajectory_) {
          ROS_WARN("AffordanceTemplate::createFromStructure() -- \'%s\' not valid, setting current trajectory to: %s", traj.c_str(), current_trajectory_.c_str());
        } else {
          ROS_INFO("AffordanceTemplate::createFromStructure() -- setting current trajectory to: %s", current_trajectory_.c_str());          
        }
        break;
      }
    }
  } 
  // still no valid traj found
  if(current_trajectory_=="") {
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
      ROS_WARN("Setting scale factor for %s", obj.name.c_str());
    } else {
      ROS_WARN("huh %s", obj.name.c_str());
    }

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
    marker.id = idx;    
  
    if(!keep_poses || !hasObjectFrame(obj.name) ) {
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = root_frame_;
      ps.pose = originToPoseMsg(obj.origin);
      frame_store_[obj.name] = FrameInfo(obj.name, ps);
      int_marker.pose = ps.pose;
    } else {
      int_marker.pose = frame_store_[obj.name].second.pose;
      if(obj.parent != "") {
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
      ROS_INFO("Drawing Mesh for object %s : %s (scale=%.3f)", obj.name.c_str(), marker.mesh_resource.c_str(), object_scale_factor_[obj.name]);
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
      control.markers[0].mesh_use_embedded_materials = false;
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
    server_->applyChanges();

    idx += 1;

  }

  return true;
}


bool AffordanceTemplate::createWaypointsFromStructure(affordance_template_object::AffordanceTemplateStructure structure, bool keep_poses) 
{

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
      // std::cout << "Display Pose in frame: " << parent_obj << ":\n" << display_pose << std::endl;
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

      tf::Transform wpTee;
      tf::Transform eeTtf;
      tf::Transform tfTm;
      tf::Transform wpTm;

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

  for(auto& o : object_menu_options_) {
    if(o.first == "Choose Trajectory") {
      setupTrajectoryMenu(structure, obj.name);
    } else {
      setupSimpleMenuItem(structure, obj.name, o.first, o.second);
    }
  }


  // for m,c in self.object_menu_options :
  //     if m == "Add Waypoint Before" or m == "Add Waypoint After":
  //         sub_menu_handle = self.marker_menus[obj].insert(m)
  //         for ee in self.robot_interface.end_effector_name_map.iterkeys() :
  //             name = self.robot_interface.end_effector_name_map[ee]
  //             self.menu_handles[(obj,m,name)] = self.marker_menus[obj].insert(name,parent=sub_menu_handle,callback=self.create_waypoint_callback)


}

void AffordanceTemplate::setupWaypointMenu(AffordanceTemplateStructure structure, std::string name)
{
  for(auto& o : waypoint_menu_options_) {
    if(o.first == "Change End-Effector Pose") {
      setupEndEffectorPoseMenu(name);
    } else {
      setupSimpleMenuItem(structure, name, o.first, o.second);
    }
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


void AffordanceTemplate::setupTrajectoryMenu(AffordanceTemplateStructure structure, const std::string& name)
{
  std::string menu_text = "Choose Trajectory";
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[name].insert( menu_text );
  for(auto &traj: structure.ee_trajectories) {
    MenuHandleKey key;
    key[name] = {menu_text, traj.name};
    group_menu_handles_[key] = marker_menus_[name].insert( sub_menu_handle, traj.name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) );   
    if(traj.name == current_trajectory_) {
      marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
    } else {
      marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );       
    }
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

void AffordanceTemplate::addInteractiveMarker(visualization_msgs::InteractiveMarker m)
{
  std::string name = m.name;
  ROS_INFO("AffordanceTemplate::addInteractiveMarker() -- %s with frame: %s", m.name.c_str(), m.header.frame_id.c_str());
  int_markers_[m.name] = m;
  server_->insert(m);
  server_->setCallback(m.name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ));
}

void AffordanceTemplate::removeInteractiveMarker(std::string marker_name) 
{
  server_->erase(marker_name);
  server_->applyChanges();
}

void AffordanceTemplate::removeAllMarkers() {
  for(auto &m: int_markers_) {
    removeInteractiveMarker(m.first);
  }
  group_menu_handles_.clear();
  int_markers_.clear();
  marker_menus_.clear();
  server_->applyChanges();
}

void AffordanceTemplate::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) 
{

  ROS_INFO("AffordanceTemplate::processFeedback() -- %s", feedback->marker_name.c_str());

  interactive_markers::MenuHandler::CheckState state;

  // set up key maps for easy comparison to menu handler ID
  MenuHandleKey reset_key;
  MenuHandleKey save_key;
  MenuHandleKey hide_controls_key;
  
  reset_key[feedback->marker_name] = {"Reset"};
  save_key[feedback->marker_name] = {"Save"};
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

    ROS_INFO("AffordanceTemplate::processFeedback() -- %s", feedback->marker_name.c_str());

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP :      
      ROS_DEBUG("AffordanceTemplate::processFeedback() --   MOUSE_UP");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT :

      ROS_DEBUG("AffordanceTemplate::processFeedback() -- MENU_SELECT");
      ROS_DEBUG("AffordanceTemplate::processFeedback() --   menu id: %d", feedback->menu_entry_id);
      // ROS_DEBUG("AffordanceTemplate::processFeedback() --   pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s", 
      //                                                             feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z, 
      //                                                             feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w,
      //                                                             feedback->header.frame_id.c_str());

      if(group_menu_handles_.find(reset_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[reset_key] == feedback->menu_entry_id) {
          ROS_DEBUG("AffordanceTemplate::processFeedback() --   RESET");
          structure_ = initial_structure_;      
          appendIDToStructure(structure_);
          removeAllMarkers();
          clearTrajectoryFlags();
          createFromStructure(structure_, false, current_trajectory_); //FIXME:: make sure current_traj is i ORIGINAL list of traj
        }
      }

      if(group_menu_handles_.find(save_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[save_key] == feedback->menu_entry_id) {
          ROS_DEBUG("AffordanceTemplate::processFeedback() --   SAVE");
        }
      }

      if(group_menu_handles_.find(hide_controls_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[hide_controls_key] == feedback->menu_entry_id) {
          ROS_DEBUG("AffordanceTemplate::processFeedback() --   CONTROLS TOGGLE");
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
      
      break;

    default :
      break;
  }
  server_->applyChanges();

}

bool AffordanceTemplate::isObject(std::string obj) {
  for(auto &o : structure_.display_objects) {
    if(o.name == obj) {
      return true;
    }
  }
  return false;
}

bool AffordanceTemplate::isWaypoint(std::string wp) {
  return !isObject(wp);
}


bool AffordanceTemplate::hasObjectFrame(std::string obj) {
  return isObject(obj) && (frame_store_.find(obj) != std::end(frame_store_)); 
}

bool AffordanceTemplate::hasWaypointFrame(std::string wp) {
  return isWaypoint(wp) && (frame_store_.find(wp) != std::end(frame_store_)); 
}

void AffordanceTemplate::run()
{
 
  ros::Rate loop_rate(loop_rate_);
  tf::Transform transform;
  FrameInfo fi;

  ros::AsyncSpinner spinner(1.0/loop_rate_);
  spinner.start();

  ROS_INFO("%s spinning.", nh_.getNamespace().c_str());
  while(ros::ok())
  {
    for(auto &f: frame_store_) {
      fi = f.second;
      tf::poseMsgToTF(fi.second.pose, transform);
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fi.second.header.frame_id, fi.first));
    }
    loop_rate.sleep();
  }

}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "affordance_template_test");
  ros::NodeHandle nh("~");
 
  std::string robot_name, template_type;
  
  if (nh.hasParam("robot_name")) {
    nh.getParam("robot_name", robot_name); 
  } else {
    robot_name = "r2_upperbody";
  }

  if (nh.hasParam("template_type")) {
    nh.getParam("template_type", template_type); 
  } else {
    template_type = "wheel";
  }

  boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface;
  robot_interface.reset(new affordance_template_markers::RobotInterface());
  robot_interface->load("r2_upperbody.yaml");
  robot_interface->configure();

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  server.reset( new interactive_markers::InteractiveMarkerServer(std::string(robot_name + "_affordance_template_server"),"",false) );

  AffordanceTemplate at(nh, server, robot_interface, robot_name, template_type, 0);
  
  AffordanceTemplateStructure structure;
  geometry_msgs::Pose p;
  at.loadFromFile("/home/swhart/ros/catkin_workspace/src/affordance_templates/affordance_template_library/templates/wheel.json", p, structure);

  at.run();
 
  return 0;
}


